#include "zed_spatial_mapper.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"

#include "zed_wrapper/Chunk.h"
#include "zed_wrapper/Normal.h"
#include "zed_wrapper/Triangle.h"
#include "zed_wrapper/UV.h"
#include "zed_wrapper/Vertex.h"

// Define if you want to use the mesh as a set of chunks or as a global entity.
#define USE_CHUNKS 1

ZedSpatialMapper::ZedSpatialMapper(std::shared_ptr<sl::Camera> zed)
{
	this->zed = zed;

	// Configure Spatial Mapping and filtering parameters
    spatial_mapping_params.range_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE_FAR);
    spatial_mapping_params.resolution_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION_LOW);
    spatial_mapping_params.save_texture = true;
    spatial_mapping_params.max_memory_usage = 512;
    spatial_mapping_params.use_chunk_only = USE_CHUNKS; // If we use chunks we do not need to keep the mesh consistent

    filter_params.set(sl::MeshFilterParameters::MESH_FILTER_LOW);

    start();
}

// Start the spatial mapping process
void ZedSpatialMapper::start()
{
    // clear previously used objects
    mesh.clear();
    mesh_msg.chunks.clear();
    mesh_msg.vertices.clear();
    mesh_msg.triangles.clear();
    mesh_msg.normals.clear();
    mesh_msg.uv.clear();

    // Enable positional tracking before starting spatial mapping
    zed->enableTracking();
    // Enable spatial mapping
    zed->enableSpatialMapping(spatial_mapping_params);

    // Start a timer, we retrieve the mesh every XXms.
    t_last = std::chrono::high_resolution_clock::now();

    is_mapping = true;
    ROS_INFO("Spatial mapping started...");
}

// Stop the spatial mapping process
void ZedSpatialMapper::stop()
{
    // Stop the mesh request and extract the whole mesh to filter it and save it as an obj file
    is_mapping = false;
    ROS_INFO("Spatial mapping stopped...");

    save();
}

void ZedSpatialMapper::save()
{
    // Extract the whole mesh
    sl::Mesh wholeMesh;
    zed->extractWholeMesh(wholeMesh);
    std::cout << ">> Mesh has been extracted..." << std::endl;

    // Filter the extracted mesh
    wholeMesh.filter(filter_params, USE_CHUNKS);
    std::cout << ">> Mesh has been filtered..." << std::endl;

    // If textures have been saved during spatial mapping, apply them to the mesh
    if (spatial_mapping_params.save_texture) {
        wholeMesh.applyTexture(sl::MESH_TEXTURE_RGB);
        std::cout << ">> Mesh has been textured..." << std::endl;
    }

    //Save as an OBJ file
    std::string saveName = "./mesh_gen.obj";
    bool t = wholeMesh.save(saveName.c_str());
    if (t) std::cout << ">> Mesh has been saved under " << saveName << std::endl;
    else std::cout << ">> Failed to save the mesh under  " << saveName << std::endl;

//     // Update the displayed Mesh
// #if USE_CHUNKS
//     mesh_object.clear();
//     mesh_object.resize(wholeMesh.chunks.size());
//     for (int c = 0; c < wholeMesh.chunks.size(); c++)
//         mesh_object[c].updateMesh(wholeMesh.chunks[c].vertices, wholeMesh.chunks[c].triangles);
// #else
//     mesh_object[0].updateMesh(wholeMesh.vertices, wholeMesh.triangles);
// #endif
}

// Update the mesh and draw image and wireframe using OpenGL
void ZedSpatialMapper::update()
{
	if (zed->grab() == sl::SUCCESS)
	{
        // Retrieve image in GPU memory
        zed->retrieveImage(left_image, sl::VIEW_LEFT, sl::MEM_GPU);

        // Update pose data (used for projection of the mesh over the current image)
        tracking_state = zed->getPosition(pose);

        if (is_mapping)
        {
            // Compute elapse time since the last call of sl::Camera::requestMeshAsync()
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_last).count();
            // Ask for a mesh update if 500ms have spend since last request
            if (duration > 500)
            {
                zed->requestMeshAsync();
                t_last = std::chrono::high_resolution_clock::now();
            }

            if (zed->getMeshRequestStatusAsync() == sl::SUCCESS)
            {
                // Get the current mesh generated and send it to opengl
                if (zed->retrieveMeshAsync(mesh) == sl::SUCCESS)
                {
                	updateMsg();
                }
            }
        }
    }
}

void ZedSpatialMapper::publish(ros::Publisher &pub_mesh)
{
	pub_mesh.publish(mesh_msg);
}

void ZedSpatialMapper::updateMsg()
{
#if USE_CHUNKS
	mesh_msg.chunks.clear();
	for (auto &chunk : mesh.chunks)
	{
		if (chunk.has_been_updated)
		{
			zed_wrapper::Chunk chunk_msg;
			for (auto &vertex : chunk.vertices)
			{
				zed_wrapper::Vertex vertex_msg;
				vertex_msg.x = vertex[0];
				vertex_msg.y = vertex[1];
				vertex_msg.z = vertex[2];
				chunk_msg.vertices.push_back(vertex_msg);
			}
			for (auto &triangle : chunk.triangles)
			{
				zed_wrapper::Triangle triangle_msg;
				triangle_msg.x = triangle[0];
				triangle_msg.y = triangle[1];
				triangle_msg.z = triangle[2];
				chunk_msg.triangles.push_back(triangle_msg);
			}
			for (auto &normal : chunk.normals)
			{
				zed_wrapper::Normal normal_msg;
				normal_msg.x = normal[0];
				normal_msg.y = normal[1];
				normal_msg.z = normal[2];
				chunk_msg.normals.push_back(normal_msg);
			}
			for (auto &uv : chunk.uv)
			{
				zed_wrapper::UV uv_msg;
				uv_msg.u = uv[0];
				uv_msg.v = uv[1];
				chunk_msg.uv.push_back(uv_msg);
			}
			mesh_msg.chunks.push_back(chunk_msg);
		}
	}
#else
	mesh_msg.vertices.clear();
	mesh_msg.triangles.clear();
	mesh_msg.normals.clear();
	mesh_msg.uv.clear();
	for (auto &vertex : mesh.vertices)
	{
		zed_wrapper::Vertex vertex_msg;
		vertex_msg.x = vertex[0];
		vertex_msg.y = vertex[1];
		vertex_msg.z = vertex[2];
		mesh_msg.vertices.push_back(vertex_msg);
	}
	for (auto &triangle : mesh.triangles)
	{
		zed_wrapper::Triangle triangle_msg;
		triangle_msg.x = triangle[0];
		triangle_msg.y = triangle[1];
		triangle_msg.z = triangle[2];
		mesh_msg.triangles.push_back(triangle_msg);
	}
	for (auto &normal : mesh.normals)
	{
		zed_wrapper::Normal normal_msg;
		normal_msg.x = normal[0];
		normal_msg.y = normal[1];
		normal_msg.z = normal[2];
		mesh_msg.normals.push_back(normal_msg);
	}
	for (auto &uv : mesh.uv)
	{
		zed_wrapper::UV uv_msg;
		uv_msg.u = uv[0];
		uv_msg.v = uv[1];
		mesh_msg.uv.push_back(uv_msg);
	}
#endif
}