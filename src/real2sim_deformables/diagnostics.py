"""Pinned Newton 1.7 detector instrumentation; preserve overflow across all passes."""

import warp as wp


@wp.kernel
def record_overflows(
    current: wp.array[wp.int32],
    vertices: wp.array[wp.int32],
    edges: wp.array[wp.int32],
    accumulated: wp.array[wp.int32],
):
    i = wp.tid()
    if i < 4:
        accumulated[i] = wp.max(accumulated[i], current[i])
    if i < vertices.shape[0]:
        wp.atomic_max(accumulated, 4, vertices[i])
    if i < edges.shape[0]:
        wp.atomic_max(accumulated, 5, edges[i])


def track_overflows(detector):
    accumulated = wp.zeros(6, dtype=wp.int32, device=detector.model.device)
    for name in ("vertex_triangle_collision_detection", "edge_edge_collision_detection"):
        original = getattr(detector, name)

        def wrapped(*args, _original=original, **kwargs):
            result = _original(*args, **kwargs)
            wp.launch(
                record_overflows,
                dim=max(4, detector.model.particle_count, detector.model.edge_count),
                inputs=[
                    detector.resize_flags,
                    detector.vertex_colliding_triangles_count,
                    detector.edge_colliding_edges_count,
                    accumulated,
                ],
                device=detector.model.device,
            )
            return result

        setattr(detector, name, wrapped)
    return accumulated
