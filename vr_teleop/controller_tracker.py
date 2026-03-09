#!/usr/bin/env python3
"""
Read 6-DOF controller poses from HTC VIVE XR Elite via OpenXR + WiVRN.

Prints grip pose (position + quaternion), trigger, and squeeze values for
both hands at ~10 Hz.  The XR frame loop runs at headset refresh rate (~90 Hz)
but pose output is throttled to keep the terminal readable.

Renders a dim background in the headset so the session stays FOCUSED.

Usage:
    python controller_tracker.py [--hz 10]
"""

import argparse
import ctypes
import signal
import sys
import time

import glfw
import numpy as np
import xr
from OpenGL import GL

_running = True


def _signal_handler(_sig, _frame):
    global _running
    _running = False


def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)

    parser = argparse.ArgumentParser(description="VR controller tracker")
    parser.add_argument("--hz", type=float, default=10.0,
                        help="Pose print rate in Hz (default: 10)")
    args = parser.parse_args()
    print_interval = 1.0 / args.hz

    # -- OpenXR instance --
    extensions = xr.enumerate_instance_extension_properties()
    required = [xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]
    for ext in required:
        if ext not in extensions:
            print(f"ERROR: Required extension {ext} not available")
            sys.exit(1)

    instance = xr.create_instance(xr.InstanceCreateInfo(
        application_info=xr.ApplicationInfo(
            application_name="Controller Tracker",
            application_version=xr.Version(0, 1, 0),
            engine_name="pyopenxr",
            engine_version=xr.Version(0, 1, 0),
            api_version=xr.Version(1, 0, 0),
        ),
        enabled_extension_names=required,
    ))
    props = xr.get_instance_properties(instance)
    print(f"Runtime: {props.runtime_name.decode()} v{xr.Version(props.runtime_version)}")

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

    # -- GLFW hidden window for GL context --
    if not glfw.init():
        raise RuntimeError("GLFW init failed")
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.DOUBLEBUFFER, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 5)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    window = glfw.create_window(64, 64, "ctrl-track", None, None)
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

    # -- Reference space (STAGE = floor-level tracking) --
    ref_space = xr.create_reference_space(session, xr.ReferenceSpaceCreateInfo(
        reference_space_type=xr.ReferenceSpaceType.STAGE,
        pose_in_reference_space=xr.Posef(),
    ))

    # -- Action set --
    action_set = xr.create_action_set(instance, xr.ActionSetCreateInfo(
        action_set_name="tracking",
        localized_action_set_name="Controller Tracking",
        priority=0,
    ))

    hand_paths = [
        xr.string_to_path(instance, "/user/hand/left"),
        xr.string_to_path(instance, "/user/hand/right"),
    ]

    grip_action = xr.create_action(action_set, xr.ActionCreateInfo(
        action_name="grip_pose",
        action_type=xr.ActionType.POSE_INPUT,
        localized_action_name="Grip Pose",
        subaction_paths=hand_paths,
    ))

    aim_action = xr.create_action(action_set, xr.ActionCreateInfo(
        action_name="aim_pose",
        action_type=xr.ActionType.POSE_INPUT,
        localized_action_name="Aim Pose",
        subaction_paths=hand_paths,
    ))

    trigger_action = xr.create_action(action_set, xr.ActionCreateInfo(
        action_name="trigger",
        action_type=xr.ActionType.FLOAT_INPUT,
        localized_action_name="Trigger",
        subaction_paths=hand_paths,
    ))

    squeeze_action = xr.create_action(action_set, xr.ActionCreateInfo(
        action_name="squeeze",
        action_type=xr.ActionType.FLOAT_INPUT,
        localized_action_name="Squeeze",
        subaction_paths=hand_paths,
    ))

    # -- Suggest bindings for known controller profiles --
    profiles = [
        ("/interaction_profiles/htc/vive_controller", {
            "grip": "/input/grip/pose",
            "aim": "/input/aim/pose",
            "trigger": "/input/trigger/value",
            "squeeze": "/input/squeeze/click",
        }),
        ("/interaction_profiles/khr/simple_controller", {
            "grip": "/input/grip/pose",
            "aim": "/input/aim/pose",
            "trigger": "/input/select/click",
            "squeeze": None,
        }),
        ("/interaction_profiles/bytedance/pico4_controller", {
            "grip": "/input/grip/pose",
            "aim": "/input/aim/pose",
            "trigger": "/input/trigger/value",
            "squeeze": "/input/squeeze/value",
        }),
    ]

    for profile_path, bindings_map in profiles:
        try:
            bindings = []
            for hand in ("/user/hand/left", "/user/hand/right"):
                bindings.append(xr.ActionSuggestedBinding(
                    action=grip_action,
                    binding=xr.string_to_path(instance, f"{hand}{bindings_map['grip']}"),
                ))
                bindings.append(xr.ActionSuggestedBinding(
                    action=aim_action,
                    binding=xr.string_to_path(instance, f"{hand}{bindings_map['aim']}"),
                ))
                bindings.append(xr.ActionSuggestedBinding(
                    action=trigger_action,
                    binding=xr.string_to_path(instance, f"{hand}{bindings_map['trigger']}"),
                ))
                if bindings_map.get("squeeze"):
                    bindings.append(xr.ActionSuggestedBinding(
                        action=squeeze_action,
                        binding=xr.string_to_path(instance, f"{hand}{bindings_map['squeeze']}"),
                    ))

            xr.suggest_interaction_profile_bindings(instance,
                xr.InteractionProfileSuggestedBinding(
                    interaction_profile=xr.string_to_path(instance, profile_path),
                    suggested_bindings=bindings,
                ))
            print(f"  Bound profile: {profile_path}")
        except xr.exception.XrException:
            pass

    # -- Action spaces --
    grip_spaces = []
    aim_spaces = []
    for hp in hand_paths:
        grip_spaces.append(xr.create_action_space(session, xr.ActionSpaceCreateInfo(
            action=grip_action, subaction_path=hp, pose_in_action_space=xr.Posef(),
        )))
        aim_spaces.append(xr.create_action_space(session, xr.ActionSpaceCreateInfo(
            action=aim_action, subaction_path=hp, pose_in_action_space=xr.Posef(),
        )))

    # Attach action sets
    xr.attach_session_action_sets(session, xr.SessionActionSetsAttachInfo(
        action_sets=[action_set],
    ))

    # -- View config + swapchains (minimal, needed for frame loop) --
    view_config_type = xr.ViewConfigurationType.PRIMARY_STEREO
    view_configs = xr.enumerate_view_configuration_views(instance, system_id, view_config_type)
    formats = xr.enumerate_swapchain_formats(session)
    preferred = [GL.GL_SRGB8_ALPHA8, GL.GL_RGBA8, GL.GL_RGBA16F]
    color_format = next((pf for pf in preferred if pf in formats), formats[0])

    swapchains = []
    swapchain_images = []
    for vc in view_configs:
        sc = xr.create_swapchain(session, xr.SwapchainCreateInfo(
            usage_flags=xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT,
            format=color_format,
            sample_count=1,
            width=vc.recommended_image_rect_width,
            height=vc.recommended_image_rect_height,
            face_count=1, array_size=1, mip_count=1,
        ))
        swapchains.append(sc)
        swapchain_images.append(
            xr.enumerate_swapchain_images(sc, xr.SwapchainImageOpenGLKHR)
        )

    fbo = GL.glGenFramebuffers(1)

    # -- Frame loop --
    session_state = xr.SessionState.UNKNOWN
    session_running = False
    last_print = 0.0
    frame_count = 0
    t0 = time.monotonic()

    print("\nWaiting for session to start...")
    print("Move controllers to see tracking data.\n")

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

        # XR frame
        frame_state = xr.wait_frame(session)
        xr.begin_frame(session)

        # Sync actions (only when session is focused)
        actions_synced = False
        if session_state == xr.SessionState.FOCUSED:
            try:
                active_sets = (xr.ActiveActionSet * 1)(
                    xr.ActiveActionSet(action_set=action_set, subaction_path=xr.NULL_PATH),
                )
                xr.sync_actions(session, xr.ActionsSyncInfo(active_action_sets=active_sets))
                actions_synced = True
            except xr.exception.SessionNotFocused:
                pass

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
                GL.glClearColor(0.05, 0.05, 0.1, 1.0)
                GL.glClear(GL.GL_COLOR_BUFFER_BIT)
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

        # Read and print controller poses at throttled rate
        now = time.monotonic()
        if actions_synced and now - last_print >= print_interval:
            last_print = now
            predicted_time = frame_state.predicted_display_time

            parts = []
            for hand_idx, label in enumerate(["L", "R"]):
                hp = hand_paths[hand_idx]

                # Grip pose
                grip_loc = xr.locate_space(
                    grip_spaces[hand_idx], ref_space, predicted_time,
                )
                flags = grip_loc.location_flags
                pos_valid = bool(flags & xr.SpaceLocationFlags.POSITION_VALID_BIT)
                ori_valid = bool(flags & xr.SpaceLocationFlags.ORIENTATION_VALID_BIT)

                if pos_valid and ori_valid:
                    p = grip_loc.pose.position
                    q = grip_loc.pose.orientation
                    pos_str = f"[{p.x:+.3f},{p.y:+.3f},{p.z:+.3f}]"
                    quat_str = f"[{q.w:+.3f},{q.x:+.3f},{q.y:+.3f},{q.z:+.3f}]"
                else:
                    pos_str = "[  n/a  ]"
                    quat_str = "[  n/a  ]"

                # Trigger
                try:
                    tr = xr.get_action_state_float(session, xr.ActionStateGetInfo(
                        action=trigger_action, subaction_path=hp,
                    ))
                    tr_val = f"{tr.current_state:.2f}" if tr.is_active else "n/a"
                except xr.exception.XrException:
                    tr_val = "n/a"

                # Squeeze
                try:
                    sq = xr.get_action_state_float(session, xr.ActionStateGetInfo(
                        action=squeeze_action, subaction_path=hp,
                    ))
                    sq_val = f"{sq.current_state:.2f}" if sq.is_active else "n/a"
                except xr.exception.XrException:
                    sq_val = "n/a"

                parts.append(f"{label}: pos={pos_str} quat={quat_str} trig={tr_val} sq={sq_val}")

            elapsed = now - t0
            fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"[{elapsed:6.1f}s {fps:.0f}fps] {' | '.join(parts)}")

    # -- Cleanup --
    elapsed = time.monotonic() - t0
    print(f"\nDone: {frame_count} frames in {elapsed:.1f}s ({frame_count/max(elapsed,0.001):.1f} fps)")

    GL.glDeleteFramebuffers(1, [fbo])
    for sp in grip_spaces + aim_spaces:
        xr.destroy_space(sp)
    xr.destroy_action_set(action_set)
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
