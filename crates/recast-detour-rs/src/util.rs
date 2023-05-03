const RC_MESH_NULL_IDX: u16 = 0xffff;

// A (nearly) direct port of rcBuildMeshAdjacency from RecastMesh.cpp.
pub fn build_mesh_adjacency(polys: &mut [u16], vertex_count: usize, vertices_per_poly: usize) {
    let poly_stride = vertices_per_poly * 2;
    let poly_count = polys.len() / poly_stride;

    let max_edge_count = polys.len() * vertices_per_poly;
    let mut edge_indices = vec![0; vertex_count + max_edge_count];
    let (first_edge, next_edge) = edge_indices.split_at_mut(vertex_count);

    #[allow(non_snake_case)]
    struct Edge {
        vert: [u16; 2],
        polyEdge: [u16; 2],
        poly: [u16; 2],
    }
    let mut edges = Vec::with_capacity(max_edge_count);

    first_edge.fill(RC_MESH_NULL_IDX);
    for i in 0..poly_count {
        let t = &polys[i * poly_stride..(i + 1) * poly_stride];
        for j in 0..vertices_per_poly {
            if t[j] == RC_MESH_NULL_IDX {
                break;
            }
            let v0 = t[j];
            let v1 = if j + 1 >= vertices_per_poly || t[j + 1] == RC_MESH_NULL_IDX {
                t[0]
            } else {
                t[j + 1]
            };
            if v0 < v1 {
                edges.push(Edge {
                    vert: [v0, v1],
                    poly: [i as u16, i as u16],
                    polyEdge: [j as u16, 0],
                });
                next_edge[edges.len() - 1] = first_edge[v0 as usize];
                first_edge[v0 as usize] = (edges.len() - 1) as u16;
            }
        }
    }

    for i in 0..poly_count {
        let t = &polys[i * poly_stride..(i + 1) * poly_stride];
        for j in 0..vertices_per_poly {
            if t[j] == RC_MESH_NULL_IDX {
                break;
            }
            let v0 = t[j];
            let v1 = if j + 1 >= vertices_per_poly || t[j + 1] == RC_MESH_NULL_IDX {
                t[0]
            } else {
                t[j + 1]
            };
            if v0 > v1 {
                let mut e = first_edge[v1 as usize];
                while e != RC_MESH_NULL_IDX {
                    let edge = &mut edges[e as usize];
                    if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
                        edge.poly[1] = i as u16;
                        edge.polyEdge[1] = j as u16;
                        break;
                    }
                    e = next_edge[e as usize];
                }
            }
        }
    }

    for e in edges.iter() {
        if e.poly[0] != e.poly[1] {
            let p0_start = e.poly[0] as usize * poly_stride;
            let p1_start = e.poly[1] as usize * poly_stride;

            polys[p0_start + vertices_per_poly + e.polyEdge[0] as usize] = e.poly[1];
            polys[p1_start + vertices_per_poly + e.polyEdge[1] as usize] = e.poly[0];
        }
    }
}
