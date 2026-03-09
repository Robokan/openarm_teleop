#!/usr/bin/env python3
"""
Minimal OpenXR hello-world: creates a session, runs the frame loop for a few
seconds rendering a solid colour into each eye, then exits cleanly.

Validates the full chain:
  Python -> OpenXR loader -> WiVRN runtime -> headset display
"""

import ctypes
import signal
import sys
import time

import glfw
import xr
from OpenGL import GL

RENDER_SECONDS = 30
CLEAR_COLOR = (0.0, 0.4, 0.7, 1.0)

_running = True


def _signal_handler(_sig, _frame):
    global _running
    _running = False


def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)

    # -- OpenXR instance --
    extensions = xr.enumerate_instance_extension_properties()
    required = [xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]
    for ext in required:
        if ext not in extensions:
            print(f"ERROR: Required extension {ext} not available")
            sys.exit(1)

    app_info = xr.ApplicationInfo(
        application_name="OpenXR Hello",
        application_version=xr.Version(0, 1, 0),
        engine_name="pyopenxr",
        engine_version=xr.Version(0, 1, 0),
        api_version=xr.Version(1, 0, 0),
    )
    instance = xr.create_instance(xr.InstanceCreateInfo(
        application_info=app_info,
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
    result = pfn(instance, system_id, ctypes.byref(gl_reqs))
    xr.check_result(xr.Result(result))
    print(f"GL requirement: {xr.Version(gl_reqs.min_api_version_supported)} - "
          f"{xr.Version(gl_reqs.max_api_version_supported)}")

    # -- GLFW hidden window for GL context --
    if not glfw.init():
        raise RuntimeError("GLFW init failed")
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.DOUBLEBUFFER, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 5)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    window = glfw.create_window(64, 64, "openxr-test", None, None)
    if window is None:
        raise RuntimeError("Failed to create GLFW window")
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

    # -- View configuration --
    view_config_type = xr.ViewConfigurationType.PRIMARY_STEREO
    view_configs = xr.enumerate_view_configuration_views(instance, system_id, view_config_type)
    print(f"Views: {len(view_configs)}")
    for i, vc in enumerate(view_configs):
        print(f"  Eye {i}: {vc.recommended_image_rect_width}x{vc.recommended_image_rect_height} "
              f"(max {vc.max_image_rect_width}x{vc.max_image_rect_height}), "
              f"samples={vc.recommended_swapchain_sample_count}")

    # -- Swapchain per eye --
    formats = xr.enumerate_swapchain_formats(session)
    preferred = [GL.GL_SRGB8_ALPHA8, GL.GL_RGBA8, GL.GL_RGBA16F, GL.GL_RGB10_A2]
    color_format = None
    for pf in preferred:
        if pf in formats:
            color_format = pf
            break
    if color_format is None:
        color_format = formats[0]
    print(f"Swapchain format: 0x{color_format:X}")

    swapchains = []
    swapchain_images = []
    for vc in view_configs:
        sc = xr.create_swapchain(session, xr.SwapchainCreateInfo(
            usage_flags=xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT | xr.SwapchainUsageFlags.SAMPLED_BIT,
            format=color_format,
            sample_count=1,
            width=vc.recommended_image_rect_width,
            height=vc.recommended_image_rect_height,
            face_count=1,
            array_size=1,
            mip_count=1,
        ))
        swapchains.append(sc)
        images = xr.enumerate_swapchain_images(sc, xr.SwapchainImageOpenGLKHR)
        swapchain_images.append(images)
        print(f"  Swapchain {len(swapchains)-1}: {len(images)} images")

    fbo = GL.glGenFramebuffers(1)

    # -- Frame loop --
    session_state = xr.SessionState.UNKNOWN
    session_running = False
    t0 = time.monotonic()
    frame_count = 0
    print(f"\nRendering solid colour for up to {RENDER_SECONDS}s ...")

    while _running and (time.monotonic() - t0) < RENDER_SECONDS:
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

        # Wait / begin frame
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
                GL.glClearColor(*CLEAR_COLOR)
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
        if frame_count % 90 == 0:
            elapsed = time.monotonic() - t0
            print(f"  {frame_count} frames, {frame_count/elapsed:.1f} fps")

    # -- Cleanup --
    elapsed = time.monotonic() - t0
    print(f"\nDone: {frame_count} frames in {elapsed:.1f}s ({frame_count/max(elapsed,0.001):.1f} fps)")

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
                    etype = xr.StructureType(event_buf.type)
                    if etype == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
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
