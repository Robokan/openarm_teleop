#!/usr/bin/env python3
"""
Stream a USB camera feed into a VR headset via OpenXR + WiVRN.

Captures frames from /dev/video0 with OpenCV in a background thread, uploads
them as OpenGL textures, and renders a fullscreen quad into each eye's swapchain
image.  Mono (single camera), so both eyes see the same flat 2D image.

The threaded capture decouples camera framerate from the XR render loop so the
headset stays at its native refresh rate even if the camera is slower.
"""

import argparse
import ctypes
import signal
import sys
import threading
import time

import cv2
import glfw
import numpy as np
import xr
from OpenGL import GL

_running = True


def _signal_handler(_sig, _frame):
    global _running
    _running = False


# ---- Threaded camera capture -----------------------------------------------

class CameraCapture:
    """Grabs camera frames in a background thread, exposing the latest one."""

    def __init__(self, device: int, width: int, height: int):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {device}")

        # Use MJPEG for higher throughput over USB
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


# ---- main ------------------------------------------------------------------

def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)

    parser = argparse.ArgumentParser(description="VR camera viewer")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument("--width", type=int, default=1920,
                        help="Capture width (default: 1920)")
    parser.add_argument("--height", type=int, default=1080,
                        help="Capture height (default: 1080)")
    args = parser.parse_args()

    # -- Camera (threaded) --
    cam = CameraCapture(args.camera, args.width, args.height)
    cam_w, cam_h = cam.width, cam.height
    print(f"Camera: {cam_w}x{cam_h} (threaded MJPEG capture)")

    # -- OpenXR instance --
    extensions = xr.enumerate_instance_extension_properties()
    required = [xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]
    for ext in required:
        if ext not in extensions:
            print(f"ERROR: Required extension {ext} not available")
            sys.exit(1)

    instance = xr.create_instance(xr.InstanceCreateInfo(
        application_info=xr.ApplicationInfo(
            application_name="VR Camera Viewer",
            application_version=xr.Version(0, 1, 0),
            engine_name="pyopenxr",
            engine_version=xr.Version(0, 1, 0),
            api_version=xr.Version(1, 0, 0),
        ),
        enabled_extension_names=required,
    ))
    props = xr.get_instance_properties(instance)
    print(f"Runtime: {props.runtime_name.decode()}")

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

    # -- GLFW hidden window --
    if not glfw.init():
        raise RuntimeError("GLFW init failed")
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.DOUBLEBUFFER, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 5)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    window = glfw.create_window(64, 64, "vr-camera", None, None)
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
        reference_space_type=xr.ReferenceSpaceType.LOCAL,
        pose_in_reference_space=xr.Posef(),
    ))

    # -- View config --
    view_config_type = xr.ViewConfigurationType.PRIMARY_STEREO
    view_configs = xr.enumerate_view_configuration_views(instance, system_id, view_config_type)

    # -- Swapchains --
    formats = xr.enumerate_swapchain_formats(session)
    preferred = [GL.GL_SRGB8_ALPHA8, GL.GL_RGBA8, GL.GL_RGBA16F]
    color_format = next((pf for pf in preferred if pf in formats), formats[0])

    swapchains = []
    swapchain_images = []
    for vc in view_configs:
        sc = xr.create_swapchain(session, xr.SwapchainCreateInfo(
            usage_flags=(xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT
                         | xr.SwapchainUsageFlags.SAMPLED_BIT),
            format=color_format,
            sample_count=1,
            width=vc.recommended_image_rect_width,
            height=vc.recommended_image_rect_height,
            face_count=1,
            array_size=1,
            mip_count=1,
        ))
        swapchains.append(sc)
        swapchain_images.append(
            xr.enumerate_swapchain_images(sc, xr.SwapchainImageOpenGLKHR)
        )

    # -- GL resources --
    fbo = GL.glGenFramebuffers(1)
    vao = GL.glGenVertexArrays(1)
    program = _create_program()
    tex_loc = GL.glGetUniformLocation(program, "tex")

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
    print("Entering frame loop (Ctrl+C to exit) ...")

    while _running:
        # Poll events
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

        # Get latest camera frame (non-blocking thanks to thread)
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

        # XR frame
        frame_state = xr.wait_frame(session)
        xr.begin_frame(session)

        layers = []
        if frame_state.should_render:
            view_state, views = xr.locate_views(session, xr.ViewLocateInfo(
                view_config_type,
                frame_state.predicted_display_time,
                ref_space,
            ))
            projection_views = (xr.CompositionLayerProjectionView * len(views))()

            for eye_idx in range(len(views)):
                sc = swapchains[eye_idx]
                img_idx = xr.acquire_swapchain_image(sc, xr.SwapchainImageAcquireInfo())
                xr.wait_swapchain_image(sc, xr.SwapchainImageWaitInfo(
                    timeout=xr.INFINITE_DURATION,
                ))
                tex = swapchain_images[eye_idx][img_idx].image
                w = view_configs[eye_idx].recommended_image_rect_width
                h = view_configs[eye_idx].recommended_image_rect_height

                GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo)
                GL.glFramebufferTexture2D(
                    GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                    GL.GL_TEXTURE_2D, tex, 0,
                )
                GL.glViewport(0, 0, w, h)
                GL.glDisable(GL.GL_DEPTH_TEST)

                GL.glUseProgram(program)
                GL.glActiveTexture(GL.GL_TEXTURE0)
                GL.glBindTexture(GL.GL_TEXTURE_2D, cam_texture)
                GL.glUniform1i(tex_loc, 0)
                GL.glBindVertexArray(vao)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
                GL.glBindVertexArray(0)
                GL.glUseProgram(0)
                GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

                xr.release_swapchain_image(sc, xr.SwapchainImageReleaseInfo())

                projection_views[eye_idx] = xr.CompositionLayerProjectionView(
                    pose=views[eye_idx].pose,
                    fov=views[eye_idx].fov,
                    sub_image=xr.SwapchainSubImage(
                        swapchain=sc,
                        image_rect=xr.Rect2Di(
                            offset=xr.Offset2Di(0, 0),
                            extent=xr.Extent2Di(w, h),
                        ),
                        image_array_index=0,
                    ),
                )

            projection_layer = xr.CompositionLayerProjection(
                space=ref_space,
                views=projection_views,
            )
            layers.append(ctypes.byref(projection_layer))

        xr.end_frame(session, xr.FrameEndInfo(
            display_time=frame_state.predicted_display_time,
            environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
            layers=layers,
        ))
        frame_count += 1
        if frame_count % 90 == 0:
            elapsed = time.monotonic() - t0
            cam_fps = cam_updates / elapsed
            print(f"  {frame_count} frames, {frame_count/elapsed:.1f} render fps, "
                  f"{cam_fps:.1f} camera fps ({cam_updates} updates)")

    # -- Cleanup --
    elapsed = time.monotonic() - t0
    print(f"\nDone: {frame_count} frames in {elapsed:.1f}s "
          f"(render {frame_count/max(elapsed,0.001):.1f} fps, "
          f"camera {cam_updates/max(elapsed,0.001):.1f} fps)")

    cam.stop()
    GL.glDeleteTextures(1, [cam_texture])
    GL.glDeleteProgram(program)
    GL.glDeleteVertexArrays(1, [vao])
    GL.glDeleteFramebuffers(1, [fbo])
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
