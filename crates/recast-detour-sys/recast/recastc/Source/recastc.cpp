#include "recastc.h"
#include <DetourNavMeshQuery.h>
#include <DetourNavMeshBuilder.h>
#include <DetourAlloc.h>

#include <memory>
#include <assert.h>

struct recastc_Query
{
	dtNavMeshQuery *q;
	dtNavMesh* mesh;
	dtQueryFilter filter;
};

static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly);

// void f()
// {
// #define TILEWID 128 //default

// #define TILEHGT 128 //default

// #define WORLD 0

// #define _SCALE 3

// #define _HGTSCALE 5

// #define MAXVerts 30000 //temporary

// #define MAXPolys 10000 //temporary

// #define MAXREFS 30000 //temporary

// 	long lSize;
// 	int numVerts, numInds, numAreas, numPolys;
// 	int i;

// 	uint *arData;			// area data
// 	float *vertData;			// read vertial Data
// 	uint *polys;			// read triangle indices

// 	// Step 1 compute bounding box

// 	float highX = 0, highY = 0, highZ = 0;

// 	for (int i = 0; i < numVerts * 3; i += 3)
// 	{

// 		if (vertData[i] > highX)
// 			highX = vertData[i];

// 		if (vertData[i + 1] > highY)
// 			highY = vertData[i + 1];

// 		if (vertData[i + 2] > highZ)
// 			highZ = vertData[i + 2];

// 		//        vertData[i]*=_SCALE; vertData[i+1]*=_HGTSCALE; vertData[i+2]*=_SCALE;
// 	}

// 	highX = roundUp(highX, 100);
// 	highY = roundUp(highY, 100);
// 	highZ = roundUp(highZ, 100);

// 	printf("Detected world bounds (%f,%f,%f)\n", highX, highY, highZ);

// 	float tileW = TILEWID, vxW;

// 	if (WORLD == 0)
// 		tileW = 102.400004; //exact increment for my world 0 exported from Unity

// 	vxW = tileW * _SCALE;

// 	int xtiles = (int)ceil(highX / tileW), ytiles = (int)ceil(highZ / tileW);

// 	dtNavMesh *mesh = new dtNavMesh();

// 	dtNavMeshParams parms;

// 	memset(&parms, 0, sizeof(dtNavMeshParams));

// 	parms.tileWidth = tileW;
// 	parms.tileHeight = tileW;

// 	parms.maxTiles = xtiles * ytiles;
// 	parms.maxPolys = 65535;

// 	memset(parms.orig, 0, 12);

// 	dtStatus initStat = mesh->init(&parms);

// 	if (!initStat)
// 	{

// 		printf("dtNavMesh Init failed!\n");
// 		exit(1);
// 	}

// 	ushort *vertRef = (ushort *)malloc(numVerts * 2);

// 	for (int yy = 0; yy < ytiles; yy++)
// 	{

// 		for (int xx = 0; xx < xtiles; xx++)
// 		{

// 			int vptr = 0, pptr = 0;

// 			float min[3], max[3];

// 			min[0] = xx * tileW;
// 			min[1] = 0;
// 			min[2] = yy * tileW;

// 			max[0] = (xx + 1) * tileW;
// 			max[1] = highY;
// 			max[2] = (yy + 1) * tileW;

// 			ushort tilePolys[MAXPolys];

// 			float tileVerts[MAXVerts];

// 			int cnt = 0, rcnt = 0;

// 			printf("Tile %d %d\n", xx, yy);

// 			for (int i = 0; i < (numVerts * 3); i += 3)
// 			{

// 				if (vertData[i] > highX)
// 					highX = vertData[i];

// 				if (vertData[i + 1] > highY)
// 					highY = vertData[i + 1];

// 				if (vertData[i + 2] > highZ)
// 					highZ = vertData[i + 2];

// 				//        vertData[i]*=_SCALE; vertData[i+1]*=_HGTSCALE; vertData[i+2]*=_SCALE;

// 				vertRef[cnt] = 0xffff;

// 				if (vertData[i] >= min[0] && vertData[i + 2] >= min[2])
// 				{

// 					if (vertData[i] <= max[0] && vertData[i + 2] <= max[2])
// 					{

// 						////                        printf("%f %f %f\n",vertData[i],vertData[i+1],vertData[i+2]);

// 						tileVerts[vptr++] = vertData[i];

// 						tileVerts[vptr++] = vertData[i + 1];

// 						tileVerts[vptr++] = vertData[i + 2];

// 						vertRef[cnt] = (ushort)rcnt; //backwards reference from vert array

// 						rcnt++;
// 					}
// 				}

// 				cnt++;
// 			}

// 			int numTileVerts = vptr / 3, numTilePolys = 0;

// 			for (int i = 0; i < numInds; i++)
// 			{

// 				int newInd = vertRef[polys[i]];

// 				if (newInd != 0xffff)
// 					tilePolys[numTilePolys++] = newInd;
// 			}

// 			ushort *doubleInds = (ushort *)malloc(numInds * 4);

// 			for (int i = 0; i < numTilePolys; i++)
// 			{

// 				doubleInds[i * 6] = tilePolys[i * 3];

// 				doubleInds[i * 6 + 1] = tilePolys[i * 3 + 1];

// 				doubleInds[i * 6 + 2] = tilePolys[i * 3 + 2];

// 				doubleInds[i * 6 + 3] = tilePolys[i * 3];

// 				doubleInds[i * 6 + 4] = tilePolys[i * 3 + 1];

// 				doubleInds[i * 6 + 5] = tilePolys[i * 3 + 2];
// 			}

// 			uchar *polyareas = (uchar *)malloc(numTilePolys);

// 			for (int i = 0; i < numTilePolys; i++)
// 				polyareas[i] = 0; //******************* passable? or FLAGS

// 			ushort *polyflags = (ushort *)malloc(numTilePolys * 2);

// 			for (int i = 0; i < numTilePolys; i++)
// 				polyflags[i] = 1; //******************* passable? or AREAS

// 			ushort *iverts = (ushort *)malloc(numTileVerts * 6);

// 			for (i = 0; i < numTileVerts; i++)
// 			{

// 				iverts[i * 3] = (ushort)(tileVerts[i * 3] * _SCALE);

// 				iverts[i * 3 + 1] = (ushort)(tileVerts[i * 3 + 1] * _HGTSCALE);

// 				iverts[i * 3 + 2] = (ushort)(tileVerts[i * 3 + 2] * _SCALE);
// 			}

// 			dtNavMeshCreateParams params;

// 			memset(&params, 0, sizeof(params));

// 			params.verts = iverts; // (uint*)vertData;

// 			params.vertCount = numTileVerts;

// 			params.polys = doubleInds; // (uint*)(data+headerSize+verts*12);

// 			params.polyCount = numTilePolys;

// 			params.polyAreas = (uchar *)polyareas;

// 			params.polyFlags = (ushort *)polyflags;

// 			params.nvp = 3;

// 			params.detailMeshes = 0;

// 			params.offMeshConAreas = 0;

// 			params.offMeshConCount = 0;

// 			params.offMeshConDir = 0;

// 			params.offMeshConFlags = 0;

// 			params.offMeshConRad = 0;

// 			params.offMeshConUserID = 0;

// 			params.offMeshConVerts = 0;

// 			params.walkableHeight = 1.8; //*_SCALE;

// 			params.walkableRadius = 0.5; //*_SCALE;

// 			params.walkableClimb = 1.2; //*_SCALE;

// 			memcpy(params.bmin, (float *)min, 12);

// 			memcpy(params.bmax, (float *)max, 12);

// 			//            params.bmin[0]=0; params.bmin[1]=0; params.bmin[2]=0;

// 			//            params.bmax[0]=10000; params.bmax[1]=600; params.bmax[2]=10000;

// 			params.cs = 1 / (float)_SCALE;

// 			params.ch = 1 / (float)_HGTSCALE;

// 			params.buildBvTree = true;

// 			unsigned char *data = NULL;
// 			int dlen = 0;

// 			if (!dtCreateNavMeshData(&params, &data, &dlen))
// 				printf("dtCreateNavMeshdata failed! skipping...\n");

// 			else
// 			{

// 				if (!data)
// 					dlen = 0;

// 				if (dlen == 0)
// 				{

// 					printf("ERROR:There was no navmesh data for tile! aborting...\n");

// 					exit(1);
// 				}

// 				////            printf("--> %d %c %c %c\n",dlen,data[0],data[1],data[2]);

// 				if (!mesh->addTile(data, dlen, DT_TILE_FREE_DATA, yy * 65536 + xx, 0))
// 				{

// 					printf("addTile failed! aborting...\n");
// 					exit(1);
// 				}
// 			}

// 			free(doubleInds);
// 			free(iverts);
// 			free(polyareas);
// 			free(polyflags);
// 		}
// 	}

// 	free(vertRef);

// 	int pathCnt, sPathCnt;

// 	dtPolyRef path[256], straightPathPolys[256];

// 	float spath[256 * 3];

// 	float startV[3] = {4520.762f, 154.73f, 5782.241f}, endV[3] = {4519.002f, 158.5f, 5793.176f}; //let's get around that fence!! :)

// 	dtQueryFilter *filter = new dtQueryFilter();

// 	filter->setIncludeFlags(1);

// 	dtNavMeshQuery *nav = new dtNavMeshQuery();

// 	nav->init(mesh, 2048);

// 	printf("about to navigate...\n");

// 	dtPolyRef m_startRef;

// 	dtPolyRef m_endRef;

// 	float m_polyPickExt[3];

// 	m_polyPickExt[0] = 2;

// 	m_polyPickExt[1] = 10;

// 	m_polyPickExt[2] = 2;

// 	nav->findNearestPoly(startV, m_polyPickExt, filter, &m_startRef, 0);

// 	if (m_startRef == 0)
// 	{
// 		printf("ERROR finding polygon near start point\n");
// 		return 1;
// 	}

// 	printf("start ref %u\n", (uint)m_startRef);

// 	nav->findNearestPoly(endV, m_polyPickExt, filter, &m_endRef, 0);

// 	if (m_endRef == 0)
// 	{
// 		printf("ERROR finding polygon near end point\n");
// 		return 1;
// 	}

// 	printf("end ref %u\n", (uint)m_endRef);

// 	//    exit(1);

// 	dtStatus stat = nav->findPath(m_startRef, m_endRef, startV, endV, filter, path, &pathCnt, 256);

// 	if (stat)
// 		printf("Nav returned path count: %d\n", pathCnt);

// 	if (pathCnt > 0)
// 	{

// 		dtStatus sstat = nav->findStraightPath(startV, endV, path, pathCnt, spath, 0, straightPathPolys, &sPathCnt, 256);

// 		printf("straight path returned: %d nodes!\n", sPathCnt);

// 		for (int i = 0; i < sPathCnt; i++)
// 		{

// 			printf("Node: (%f,%f,%f)\n", spath[i * 3], spath[i * 3 + 1], spath[i * 3 + 2]);
// 		}
// 	}

// 	return 0;
// }

/// Reference Doc:
/// * A discussion how to NavMesh from pre-cull triangles soup
/// 	* https://groups.google.com/forum/#!topic/recastnavigation/Tjq7G-KUxt8

static void write_error(const char* msg, recastc_Error* error) {
	assert(strlen(msg) < 255);
	strcpy(error->msg, msg);
}

#define RETURN_ERROR(MSG)		do {  write_error(MSG, error); return 0;  } while(0)
#define POLYFLAGS_WALK			0x1

extern "C"
{	
	const char *recastc_version()
	{
		return "0.0.1";
	}

	/// Create Query Objects
	struct recastc_Query *recastc_create_query(recastc_NavMesh* qparam, recastc_Error* error)
	{
		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));

		/// NOTE:
		/// NavMesh use unsigned short to store position data
		/// Which means we have to normalize a float to unsigned short
		int pm_vert_count = qparam->vert_count;							///< The number vertices in the polygon mesh. [Limit: >= 3]
		int pm_nvp = 3;												    ///< Number maximum number of vertices per polygon. [Limit: >= 3]
		int pm_polyCount = qparam->triangles_count;					    ///< Number of polygons in the mesh. [Limit: >= 1]
		auto pm_verts = qparam->verts;								    ///< The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
		
		auto pm_polys = std::unique_ptr<unsigned short[]>(new unsigned short[pm_polyCount * 2 * pm_nvp]);  ///< The polygon data. [Size: #polyCount * 2 * #nvp]		
		
		// fill the first part of the pm_polys with triangle indices
		for (int i = 0; i < qparam->triangles_count; i++)
		{
			pm_polys[i*pm_nvp*2+0] = qparam->indices[i*pm_nvp+0];
			pm_polys[i*pm_nvp*2+1] = qparam->indices[i*pm_nvp+1];
			pm_polys[i*pm_nvp*2+2] = qparam->indices[i*pm_nvp+2];
		}
		if(!buildMeshAdjacency(pm_polys.get(), pm_polyCount, pm_vert_count, pm_nvp)) {
			RETURN_ERROR("dtCreateNavMeshdata failed!");
		}

		// TODO(edwin):
		// Fill pm_poly		
		auto pm_polyFlags = std::unique_ptr<unsigned short[]>(new unsigned short[pm_polyCount]);		    ///< The user defined flags assigned to each polygon. [Size: #polyCount]
		memset(pm_polyFlags.get(), 0, sizeof(unsigned short) * pm_polyCount);		
		
		// Referece: Sample_SoloMesh.cpp Line:667
		for(auto i = 0; i < pm_polyCount; i++)
			pm_polyFlags[i] = POLYFLAGS_WALK;


		auto pm_polyAreas = std::unique_ptr<unsigned char[]>(new unsigned char[pm_polyCount]);		    ///< The user defined area ids assigned to each polygon. [Size: #polyCount]		
		// TODO(edwin):
		// We assume all polygon are in the same area
		memset(pm_polyAreas.get(), 0, sizeof(unsigned char) * pm_polyCount);		

		params.verts = pm_verts;
		params.vertCount = pm_vert_count;
		params.polys = pm_polys.get();
		params.polyCount = pm_polyCount;
		params.polyAreas = pm_polyAreas.get();
		params.polyFlags = pm_polyFlags.get();
		params.nvp = pm_nvp;
		params.detailMeshes = 0;
		params.offMeshConAreas = 0;
		params.offMeshConCount = 0;
		params.offMeshConDir = 0;
		params.offMeshConFlags = 0;
		params.offMeshConRad = 0;
		params.offMeshConUserID = 0;
		params.offMeshConVerts = 0;
		params.walkableHeight = qparam->walkable_height;
		params.walkableRadius = qparam->walkable_radius;
		params.walkableClimb = qparam->walkable_climb;
		params.cs = qparam->cell_size;
		params.ch = qparam->cell_height;
		memcpy(params.bmin, &qparam->bmin, sizeof(params.bmin));
		memcpy(params.bmax, &qparam->bmax, sizeof(params.bmax));
		params.buildBvTree = true;

		unsigned char* navData = 0;
		int navDataSize = 0;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
			RETURN_ERROR("dtCreateNavMeshdata failed!");
		}

		if(navData == NULL) {
			RETURN_ERROR("dtCreateNavMeshdata return null data");
		}
		if(navData == NULL) {
			RETURN_ERROR("dtCreateNavMeshdata return zero size data");
		}

		dtNavMesh *mesh = dtAllocNavMesh();

		dtStatus status = mesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			dtFreeNavMesh(mesh);

			RETURN_ERROR("Could not init Detour navmesh");
		}

		auto q = dtAllocNavMeshQuery();
		status = q->init(mesh, 1000);
		if (dtStatusFailed(status))
		{
			dtFreeNavMeshQuery(q);
			dtFreeNavMesh(mesh);		

			RETURN_ERROR("Could not init Detour navmesh");
		}
			
		auto query = new recastc_Query();
		query->q = q;
		query->mesh = mesh;

		return query;
	}

	void recastc_free_query(struct recastc_Query *query)
	{
		dtFreeNavMeshQuery(query->q);
		dtFreeNavMesh(query->mesh);		
		delete query;
	}

	int32_t recastc_find_nearest_point(struct recastc_Query* query, 
        const recastc_NearestPoint* input, 
        recastc_NearestPointResult* result,
        recastc_Error* error) 
	{
		assert(query);
		assert(query->q);
		assert(query);
		assert(input);
		assert(result);		
		assert(sizeof(dtPolyRef) == sizeof(uint32_t));

		query->filter.setIncludeFlags(POLYFLAGS_WALK);

		float nearestPt[4];
		dtStatus status = query->q->findNearestPoly(input->center, input->half_extents, &query->filter, &result->poly, result->point);

		if (dtStatusFailed(status))
		{
			RETURN_ERROR("Fail to find nearest poly");
		}

		return 1;
	}
}

struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

static const unsigned short RC_MESH_NULL_IDX = 0xffff;

/// Straight up copy & paste from RecastMesh.cpp
/// We assume `polys`
static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly)
{
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php
	
	int maxEdgeCount = npolys*vertsPerPoly;
	unsigned short* firstEdge = (unsigned short*)dtAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), DT_ALLOC_TEMP);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;
	
	rcEdge* edges = (rcEdge*)dtAlloc(sizeof(rcEdge)*maxEdgeCount, DT_ALLOC_TEMP);
	if (!edges)
	{
		dtFree(firstEdge);
		return false;
	}
	
	for (int i = 0; i < nverts; i++)
		firstEdge[i] = RC_MESH_NULL_IDX;
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						break;
					}
				}
			}
		}
	}
	
	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			unsigned short* p1 = &polys[e.poly[1]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
			p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
		}
	}
	
	dtFree(firstEdge);
	dtFree(edges);
	
	return true;
}
