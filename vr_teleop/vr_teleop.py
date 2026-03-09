#!/usr/bin/env python3
"""
VR teleoperation: camera feed in headset + 6-DOF controller tracking.

Streams a USB camera into the VR display and reads controller poses that
can be mapped to robot arm joint targets.

Controller poses are printed each frame and can be consumed by a downstream
robot control pipeline via the ControllerTracker class.
"""

import argparse
import ctypes
import signal
import sys
import threading
import time
from dataclasses import dataclass, field

import cv2
import glfw
import numpy as np
import xr
from OpenGL import GL

_running = True


def _signal_handler(_sig, _frame):
    global _running
    _running = False


# ---- Data classes ----------------------------------------------------------

@dataclass
class Pose6DOF:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 1.0]))
    valid: bool = False

    def __repr__(self):
        if not self.valid:
            return "Pose6DOF(invalid)"
        p = self.position
        q = self.orientation
        return f"Pose6DOF(pos=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}] quat=[{q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}])"


@dataclass
class ControllerState:
    grip_pose: Pose6DOF = field(default_factory=Pose6DOF)
    aim_pose: Pose6DOF = field(default_factory=Pose6DOF)
    squeeze_value: float = 0.0
    trigger_value: float = 0.0
    active: bool = False


# ---- Threaded camera capture -----------------------------------------------

class CameraCapture:
    """Grabs camera frames in a background thread, exposing the latest one."""

    def __init__(self, device: int, width: int, height: int):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {device}")

        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Cannot read from camera")
        self.height, self.width = frame.shape[:2]

        self._frame = frame
        self._new_frame = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._running = True
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.001)
                continue
            with self._lock:
                self._frame = frame
                self._new_frame = True

    def get_frame(self):
        """Return (frame_copy, is_new). Frame is always valid; is_new indicates update."""
        with self._lock:
            new = self._new_frame
            self._new_frame = False
            frame = self._frame.copy()
        return frame, new

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self.cap.release()


# ---- OpenGL helpers --------------------------------------------------------

QUAD_VERT = """
#version 450 core
out vec2 uv;
void main() {
    uv = vec2((gl_VertexID & 1) * 2.0, (gl_VertexID & 2) * 1.0);
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
"""

QUAD_FRAG = """
#version 450 core
in vec2 uv;
out vec4 frag;
uniform sampler2D tex;
void main() {
    frag = texture(tex, vec2(uv.x, 1.0 - uv.y));
}
"""


def _compile_shader(source: str, shader_type: int) -> int:
    shader = GL.glCreateShader(shader_type)
    GL.glShaderSource(shader, source)
    GL.glCompileShader(shader)
    if not GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS):
        log = GL.glGetShaderInfoLog(shader).decode()
        raise RuntimeError(f"Shader compile error: {log}")
    return shader


def _create_program() -> int:
    vs = _compile_shader(QUAD_VERT, GL.GL_VERTEX_SHADER)
    fs = _compile_shader(QUAD_FRAG, GL.GL_FRAGMENT_SHADER)
    prog = GL.glCreateProgram()
    GL.glAttachShader(prog, vs)
    GL.glAttachShader(prog, fs)
    GL.glLinkProgram(prog)
    if not GL.glGetProgramiv(prog, GL.GL_LINK_STATUS):
        log = GL.glGetProgramInfoLog(prog).decode()
        raise RuntimeError(f"Program link error: {log}")
    GL.glDeleteShader(vs)
    GL.glDeleteShader(fs)
    return prog


# ---- Controller tracker ----------------------------------------------------

class ControllerTracker:
    """Manages OpenXR action sets for reading controller poses and inputs."""

    def __init__(self, instance: xr.Instance, session: xr.Session, ref_space: xr.Space):
        self.instance = instance
        self.session = session
        self.ref_space = ref_space
        self.left = ControllerState()
        self.right = ControllerState()

        self.action_set = xr.create_action_set(instance, xr.ActionSetCreateInfo(
            action_set_name="controller_tracking",
            localized_action_set_name="Controller Tracking",
            priority=0,
        ))

        self.hand_paths = [
            xr.string_to_path(instance, "/user/hand/left"),
            xr.string_to_path(instance, "/user/hand/right"),
        ]

        self.grip_action = xr.create_action(self.action_set, xr.ActionCreateInfo(
            action_name="grip_pose",
            action_type=xr.ActionType.POSE_INPUT,
            localized_action_name="Grip Pose",
            subaction_paths=self.hand_paths,
        ))

        self.aim_action = xr.create_action(self.action_set, xr.ActionCreateInfo(
            action_name="aim_pose",
            action_type=xr.ActionType.POSE_INPUT,
            localized_action_name="Aim Pose",
            subaction_paths=self.hand_paths,
        ))

        self.squeeze_action = xr.create_action(self.action_set, xr.ActionCreateInfo(
            action_name="squeeze",
            action_type=xr.ActionType.FLOAT_INPUT,
            localized_action_name="Squeeze",
            subaction_paths=self.hand_paths,
        ))

        self.trigger_action = xr.create_action(self.action_set, xr.ActionCreateInfo(
            action_name="trigger",
            action_type=xr.ActionType.FLOAT_INPUT,
            localized_action_name="Trigger",
            subaction_paths=self.hand_paths,
        ))

        self._suggest_bindings()

        self.grip_spaces = []
        self.aim_spaces = []
        for hand_path in self.hand_paths:
            self.grip_spaces.append(xr.create_action_space(session, xr.ActionSpaceCreateInfo(
                action=self.grip_action, subaction_path=hand_path,
                pose_in_action_space=xr.Posef(),
            )))
            self.aim_spaces.append(xr.create_action_space(session, xr.ActionSpaceCreateInfo(
                action=self.aim_action, subaction_path=hand_path,
                pose_in_action_space=xr.Posef(),
            )))

        xr.attach_session_action_sets(session, xr.SessionActionSetsAttachInfo(
            action_sets=[self.action_set],
        ))

    def _suggest_bindings(self):
        profiles = [
            ("/interaction_profiles/htc/vive_controller", {
                "trigger": "/input/trigger/value",
                "squeeze": "/input/squeeze/click",
            }),
            ("/interaction_profiles/khr/simple_controller", {
                "trigger": "/input/select/click",
                "squeeze": None,
            }),
        ]
        for profile_path, extras in profiles:
            try:
                bindings = []
                for hand in ("/user/hand/left", "/user/hand/right"):
                    bindings.append(xr.ActionSuggestedBinding(
                        action=self.grip_action,
                        binding=xr.string_to_path(self.instance, f"{hand}/input/grip/pose"),
                    ))
                    bindings.append(xr.ActionSuggestedBinding(
                        action=self.aim_action,
                        binding=xr.string_to_path(self.instance, f"{hand}/input/aim/pose"),
                    ))
                    bindings.append(xr.ActionSuggestedBinding(
                        action=self.trigger_action,
                        binding=xr.string_to_path(self.instance, f"{hand}{extras['trigger']}"),
                    ))
                    if extras.get("squeeze"):
                        bindings.append(xr.ActionSuggestedBinding(
                            action=self.squeeze_action,
                            binding=xr.string_to_path(self.instance, f"{hand}{extras['squeeze']}"),
                        ))
                xr.suggest_interaction_profile_bindings(self.instance,
                    xr.InteractionProfileSuggestedBinding(
                        interaction_profile=xr.string_to_path(self.instance, profile_path),
                        suggested_bindings=bindings,
                    ))
                print(f"  Bound profile: {profile_path}")
            except xr.exception.XrException:
                pass

    def update(self, predicted_time: int):
        """Sync actions and update controller states."""
        active_sets = (xr.ActiveActionSet * 1)(
            xr.ActiveActionSet(action_set=self.action_set, subaction_path=xr.NULL_PATH),
        )
        xr.sync_actions(self.session, xr.ActionsSyncInfo(active_action_sets=active_sets))

        for hand_idx, state in enumerate([self.left, self.right]):
            hand_path = self.hand_paths[hand_idx]

            grip_loc = xr.locate_space(
                self.grip_spaces[hand_idx], self.ref_space, predicted_time,
            )
            flags = grip_loc.location_flags
            if (flags & xr.SpaceLocationFlags.POSITION_VALID_BIT
                    and flags & xr.SpaceLocationFlags.ORIENTATION_VALID_BIT):
                p = grip_loc.pose.position
                q = grip_loc.pose.orientation
                state.grip_pose.position[:] = [p.x, p.y, p.z]
                state.grip_pose.orientation[:] = [q.x, q.y, q.z, q.w]
                state.grip_pose.valid = True
                state.active = True
            else:
                state.grip_pose.valid = False

            aim_loc = xr.locate_space(
                self.aim_spaces[hand_idx], self.ref_space, predicted_time,
            )
            flags = aim_loc.location_flags
            if (flags & xr.SpaceLocationFlags.POSITION_VALID_BIT
                    and flags & xr.SpaceLocationFlags.ORIENTATION_VALID_BIT):
                p = aim_loc.pose.position
                q = aim_loc.pose.orientation
                state.aim_pose.position[:] = [p.x, p.y, p.z]
                state.aim_pose.orientation[:] = [q.x, q.y, q.z, q.w]
                state.aim_pose.valid = True
            else:
                state.aim_pose.valid = False

            try:
                sq = xr.get_action_state_float(self.session, xr.ActionStateGetInfo(
                    action=self.squeeze_action, subaction_path=hand_path,
                ))
                state.squeeze_value = sq.current_state if sq.is_active else 0.0
            except xr.exception.XrException:
                state.squeeze_value = 0.0

            try:
                tr = xr.get_action_state_float(self.session, xr.ActionStateGetInfo(
                    action=self.trigger_action, subaction_path=hand_path,
                ))
                state.trigger_value = tr.current_state if tr.is_active else 0.0
            except xr.exception.XrException:
                state.trigger_value = 0.0


# ---- main ------------------------------------------------------------------

def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)

    parser = argparse.ArgumentParser(description="VR Teleop: camera + controllers")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument("--width", type=int, default=1920,
                        help="Capture width (default: 1920)")
    parser.add_argument("--height", type=int, default=1080,
                        help="Capture height (default: 1080)")
    parser.add_argument("--no-camera", action="store_true",
                        help="Run without camera (controller tracking only)")
    args = parser.parse_args()

    # -- Camera (threaded) --
    cam = None
    cam_w, cam_h = args.width, args.height
    if not args.no_camera:
        try:
            cam = CameraCapture(args.camera, args.width, args.height)
            cam_w, cam_h = cam.width, cam.height
            print(f"Camera: {cam_w}x{cam_h} (threaded MJPEG capture)", flush=True)
        except RuntimeError as e:
            print(f"WARNING: {e}, running without camera", flush=True)
            cam = None

    # -- OpenXR instance --
    extensions = xr.enumerate_instance_extension_properties()
    required = [xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]
    for ext in required:
        if ext not in extensions:
            print(f"ERROR: Required extension {ext} not available")
            sys.exit(1)

    instance = xr.create_instance(xr.InstanceCreateInfo(
        application_info=xr.ApplicationInfo(
            application_name="VR Teleop",
            application_version=xr.Version(0, 1, 0),
            engine_name="pyopenxr",
            engine_version=xr.Version(0, 1, 0),
            api_version=xr.Version(1, 0, 0),
        ),
        enabled_extension_names=required,
    ))
    props = xr.get_instance_properties(instance)
    print(f"Runtime: {props.runtime_name.decode()}", flush=True)

    # -- System --
    system_id = xr.get_system(instance, xr.SystemGetInfo(
        form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY,
    ))

    # -- GL requirements --
    pfn = ctypes.cast(
        xr.get_instance_proc_addr(instance, "xrGetOpenGLGraphicsRequirementsKHR"),
        xr.PFN_xrGetOpenGLGraphicsRequirementsKHR,
    )
    gl_reqs = xr.GraphicsRequirementsOpenGLKHR()
    xr.check_result(xr.Result(pfn(instance, system_id, ctypes.byref(gl_reqs))))

    # -- GLFW --
    if not glfw.init():
        raise RuntimeError("GLFW init failed")
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.DOUBLEBUFFER, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 5)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    window = glfw.create_window(64, 64, "vr-teleop", None, None)
    if window is None:
        raise RuntimeError("GLFW window creation failed")
    glfw.make_context_current(window)
    glfw.swap_interval(0)

    # -- Graphics binding --
    from OpenGL import GLX
    graphics_binding = xr.GraphicsBindingOpenGLXlibKHR(
        x_display=GLX.glXGetCurrentDisplay(),
        glx_drawable=GLX.glXGetCurrentDrawable(),
        glx_context=GLX.glXGetCurrentContext(),
    )

    # -- Session --
    session = xr.create_session(instance, xr.SessionCreateInfo(
        next=ctypes.cast(ctypes.pointer(graphics_binding), ctypes.c_void_p),
        system_id=system_id,
    ))

    # -- Reference space --
    ref_space = xr.create_reference_space(session, xr.ReferenceSpaceCreateInfo(
        reference_space_type=xr.ReferenceSpaceType.STAGE,
        pose_in_reference_space=xr.Posef(),
    ))

    # -- Controller tracker --
    tracker = ControllerTracker(instance, session, ref_space)

    # -- View config (still needed for begin_session) --
    view_config_type = xr.ViewConfigurationType.PRIMARY_STEREO

    # -- Single swapchain for quad layer (flat 2D panel) --
    formats = xr.enumerate_swapchain_formats(session)
    preferred = [GL.GL_SRGB8_ALPHA8, GL.GL_RGBA8, GL.GL_RGBA16F]
    color_format = next((pf for pf in preferred if pf in formats), formats[0])

    quad_w = cam_w if cam is not None else 1920
    quad_h = cam_h if cam is not None else 1080
    quad_sc = xr.create_swapchain(session, xr.SwapchainCreateInfo(
        usage_flags=(xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT
                     | xr.SwapchainUsageFlags.SAMPLED_BIT),
        format=color_format,
        sample_count=1,
        width=quad_w,
        height=quad_h,
        face_count=1, array_size=1, mip_count=1,
    ))
    quad_images = xr.enumerate_swapchain_images(quad_sc, xr.SwapchainImageOpenGLKHR)
    print(f"Quad layer: {quad_w}x{quad_h}, {len(quad_images)} images", flush=True)

    # -- GL resources --
    fbo = GL.glGenFramebuffers(1)
    vao = GL.glGenVertexArrays(1)
    program = _create_program()
    tex_loc = GL.glGetUniformLocation(program, "tex")

    cam_texture = None
    if cam is not None:
        cam_texture = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, cam_texture)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, cam_w, cam_h, 0,
                        GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, None)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

    # -- Frame loop --
    session_state = xr.SessionState.UNKNOWN
    session_running = False
    frame_count = 0
    cam_updates = 0
    t0 = time.monotonic()
    print("Entering frame loop (Ctrl+C to exit) ...", flush=True)

    while _running:
        while True:
            try:
                event_buf = xr.poll_event(instance)
                etype = xr.StructureType(event_buf.type)
                if etype == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                    ev = ctypes.cast(
                        ctypes.byref(event_buf),
                        ctypes.POINTER(xr.EventDataSessionStateChanged),
                    ).contents
                    session_state = xr.SessionState(ev.state)
                    print(f"  Session state -> {session_state.name}")
                    if session_state == xr.SessionState.READY:
                        xr.begin_session(session, xr.SessionBeginInfo(view_config_type))
                        session_running = True
                    elif session_state in (xr.SessionState.STOPPING,
                                           xr.SessionState.LOSS_PENDING,
                                           xr.SessionState.EXITING):
                        _running = False
                        break
            except xr.EventUnavailable:
                break

        if not session_running or not _running:
            time.sleep(0.01)
            continue

        # Get latest camera frame (non-blocking)
        has_frame = False
        if cam is not None:
            frame, is_new = cam.get_frame()
            if is_new:
                rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                rgba = np.ascontiguousarray(rgba)
                GL.glBindTexture(GL.GL_TEXTURE_2D, cam_texture)
                GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
                GL.glPixelStorei(GL.GL_UNPACK_ROW_LENGTH, 0)
                GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, cam_w, cam_h,
                                   GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, rgba)
                GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
                cam_updates += 1
                has_frame = True
            elif cam_updates > 0:
                has_frame = True

        # XR frame
        frame_state = xr.wait_frame(session)
        xr.begin_frame(session)

        # Update controller tracking (only when focused)
        if session_state == xr.SessionState.FOCUSED:
            try:
                tracker.update(frame_state.predicted_display_time)
            except xr.exception.SessionNotFocused:
                pass

        layers = []
        if frame_state.should_render:
            # Render camera into single quad swapchain
            img_idx = xr.acquire_swapchain_image(quad_sc, xr.SwapchainImageAcquireInfo())
            xr.wait_swapchain_image(quad_sc, xr.SwapchainImageWaitInfo(
                timeout=xr.INFINITE_DURATION,
            ))
            tex = quad_images[img_idx].image

            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo)
            GL.glFramebufferTexture2D(
                GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                GL.GL_TEXTURE_2D, tex, 0,
            )
            GL.glViewport(0, 0, quad_w, quad_h)
            GL.glDisable(GL.GL_DEPTH_TEST)

            if has_frame and cam_texture is not None:
                GL.glUseProgram(program)
                GL.glActiveTexture(GL.GL_TEXTURE0)
                GL.glBindTexture(GL.GL_TEXTURE_2D, cam_texture)
                GL.glUniform1i(tex_loc, 0)
                GL.glBindVertexArray(vao)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
                GL.glBindVertexArray(0)
                GL.glUseProgram(0)
            else:
                GL.glClearColor(0.05, 0.05, 0.1, 1.0)
                GL.glClear(GL.GL_COLOR_BUFFER_BIT)

            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
            xr.release_swapchain_image(quad_sc, xr.SwapchainImageReleaseInfo())

            # Quad layer: flat panel 2m wide, 2m in front of user at eye height
            aspect = quad_w / quad_h
            panel_width = 2.5
            panel_height = panel_width / aspect
            quad_layer = xr.CompositionLayerQuad(
                layer_flags=xr.CompositionLayerFlags.BLEND_TEXTURE_SOURCE_ALPHA_BIT,
                space=ref_space,
                eye_visibility=xr.EyeVisibility.BOTH,
                sub_image=xr.SwapchainSubImage(
                    swapchain=quad_sc,
                    image_rect=xr.Rect2Di(
                        offset=xr.Offset2Di(0, 0),
                        extent=xr.Extent2Di(quad_w, quad_h),
                    ),
                    image_array_index=0,
                ),
                pose=xr.Posef(
                    orientation=xr.Quaternionf(0, 0, 0, 1),
                    position=xr.Vector3f(0, 1.2, -2.0),
                ),
                size=xr.Extent2Df(panel_width, panel_height),
            )
            layers.append(ctypes.byref(quad_layer))

        xr.end_frame(session, xr.FrameEndInfo(
            display_time=frame_state.predicted_display_time,
            environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
            layers=layers,
        ))
        frame_count += 1

        if frame_count % 90 == 0:
            elapsed = time.monotonic() - t0
            fps = frame_count / elapsed
            cam_fps = cam_updates / elapsed if cam is not None else 0
            parts = [f"{frame_count}f {fps:.0f}fps"]
            if cam is not None:
                parts.append(f"cam {cam_fps:.0f}fps")
            if tracker.left.active:
                lp = tracker.left.grip_pose.position
                parts.append(f"L=[{lp[0]:+.2f},{lp[1]:+.2f},{lp[2]:+.2f}] tr={tracker.left.trigger_value:.1f}")
            if tracker.right.active:
                rp = tracker.right.grip_pose.position
                parts.append(f"R=[{rp[0]:+.2f},{rp[1]:+.2f},{rp[2]:+.2f}] tr={tracker.right.trigger_value:.1f}")
            print(f"  {' | '.join(parts)}")

    # -- Cleanup --
    elapsed = time.monotonic() - t0
    print(f"\nDone: {frame_count} frames in {elapsed:.1f}s")

    if cam is not None:
        cam.stop()
    if cam_texture is not None:
        GL.glDeleteTextures(1, [cam_texture])
    GL.glDeleteProgram(program)
    GL.glDeleteVertexArrays(1, [vao])
    GL.glDeleteFramebuffers(1, [fbo])
    for sp in tracker.grip_spaces + tracker.aim_spaces:
        xr.destroy_space(sp)
    xr.destroy_action_set(tracker.action_set)
    for sc in swapchains:
        xr.destroy_swapchain(sc)
    xr.destroy_space(ref_space)
    if session_running:
        try:
            xr.request_exit_session(session)
            while True:
                try:
                    event_buf = xr.poll_event(instance)
                    if xr.StructureType(event_buf.type) == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                        ev = ctypes.cast(
                            ctypes.byref(event_buf),
                            ctypes.POINTER(xr.EventDataSessionStateChanged),
                        ).contents
                        if xr.SessionState(ev.state) == xr.SessionState.STOPPING:
                            xr.end_session(session)
                            break
                except xr.EventUnavailable:
                    time.sleep(0.01)
        except xr.exception.SessionNotRunningError:
            pass
    xr.destroy_session(session)
    xr.destroy_instance(instance)
    glfw.terminate()
    print("Exited cleanly.")


if __name__ == "__main__":
    main()
