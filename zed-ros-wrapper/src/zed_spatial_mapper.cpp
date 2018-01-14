#include "zed_spatial_mapper.hpp"

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
}

// Start the spatial mapping process
void ZedSpatialMapper::start()
{
    // clear previously used objects
    mesh.clear();
    // mesh_object.clear();

#if !USE_CHUNKS
    // Create only one object that will contain the full mesh.
    // Otherwise, different MeshObject will be created for each chunk when needed
    // mesh_object.emplace_back();
#endif

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
// #if USE_CHUNKS
//                     for (int c = 0; c < mesh.chunks.size(); c++)
//                     {
//                         // If the chunk does not exist in the rendering process -> add it in the rendering list
//                         if (mesh_object.size() < mesh.chunks.size()) mesh_object.emplace_back();
//                         // If the chunck has been updated by the spatial mapping, update it for rendering
//                         if (mesh.chunks[c].has_been_updated)
//                             mesh_object[c].updateMesh(mesh.chunks[c].vertices, mesh.chunks[c].triangles);
//                     }
// #else
//                     mesh_object[0].updateMesh(mesh.vertices, mesh.triangles);
// #endif
                }
            }
        }
    }
}

void ZedSpatialMapper::publish(ros::Publisher &pub_mesh)
{
	// Not yet implemented
}