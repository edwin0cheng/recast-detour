#ifndef __RECASTC_HH
#define __RECASTC_HH

#include <stdint.h>

extern "C"
{
    typedef struct  {
        uint16_t *verts;                ///< in voxel unit
        uint32_t vert_count;           
        uint16_t *indices;              
        uint32_t triangles_count;
        float bmin[3];                  ///< NavMesh minimum bounds in world unit
        float bmax[3];                  ///< NavMesh maximum bounds in world unit

        float walkable_height;          ///< The agent height in world unit
        float walkable_radius;          ///< The agent radius in world unit
        float walkable_climb;           ///< The agent maximum traversable ledge in world unit
        float cell_size;				///< The xz-plane cell size of the polygon mesh in world unit
	    float cell_height;				///< The y-axis cell height of the polygon mesh in world unit
    } recastc_NavMesh;

    typedef struct {
        float center[3];
        float half_extents[3];
    } recastc_NearestPointInput;

    typedef struct {
        float pos[3];
        uint32_t poly;
    }  recastc_NearestPointResult;

    typedef struct {
        float pos[3];
        uint32_t poly;
    } recastc_ClosestPointInput;

    typedef struct {
        float pos[3];
    } recastc_ClosestPointResult;

    typedef struct {
        uint32_t start_poly;
        uint32_t end_poly;        
        float start_pos[3];
        float end_pos[3];    
    } recastc_PathInput;

    typedef struct {
        uint32_t path[100];        
        uint32_t path_count;
    } recastc_PathResult;

    typedef struct {
        // FIXME: Just random choose size
        char msg[256];                  
    } recastc_Error;

    const char *recastc_version();

    struct recastc_Query *recastc_create_query(recastc_NavMesh* qparam, recastc_Error* error);
    
    /// This function returns the nearst point without using the details mesh
    int32_t recastc_find_nearest_point(struct recastc_Query* query, 
        const recastc_NearestPointInput* input, 
        recastc_NearestPointResult* result,
        recastc_Error* error);

    /// This function return the closest point with details mesh
    int32_t recastc_find_closest_point(struct recastc_Query* query, 
        const recastc_ClosestPointInput* input,
        recastc_ClosestPointResult* result,
        recastc_Error* error);
    
    int32_t recastc_find_path(struct recastc_Query* query, 
        const recastc_PathInput* input,
        recastc_PathResult* result,
        recastc_Error* error);


    void recastc_free_query(struct recastc_Query* query);
}

#endif