#include "zed_spatial_mapper.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"

#include "zed_wrapper/Chunk.h"
#include "zed_wrapper/Normal.h"
#include "zed_wrapper/Triangle.h"
#include "zed_wrapper/UV.h"
#include "zed_wrapper/Vertex.h"

ZedSpatialMapper::ZedSpatialMapper(std::shared_ptr<sl::Camera> zed)
{
	this->zed = zed;

	// Configure Spatial Mapping and filtering parameters
    spatial_mapping_params.range_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE_FAR);
    spatial_mapping_params.resolution_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION_LOW);
    spatial_mapping_params.save_texture = true;
    spatial_mapping_params.max_memory_usage = 512;
    spatial_mapping_params.use_chunk_only = true; // If we use chunks we do not need to keep the mesh consistent

    filter_params.set(sl::MeshFilterParameters::MESH_FILTER_LOW);
}

// Start the spatial mapping process
void ZedSpatialMapper::start()
{
    // clear previously used objects
    mesh.clear();
    mesh_msg.chunks.clear();

    // Positional tracking has already been enabled at this point, so enable
    // spatial mapping
    zed->enableSpatialMapping(spatial_mapping_params);

    // Start a timer, we retrieve the mesh every XXms.
    t_last = std::chrono::high_resolution_clock::now();

    perspective = createPerspective();

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
    ROS_INFO("Mesh has been extracted...");

    // Filter the extracted mesh
    wholeMesh.filter(filter_params, true);
    ROS_INFO("Mesh has been filtered...");

    // If textures have been saved during spatial mapping, apply them to the mesh
    if (spatial_mapping_params.save_texture) {
        wholeMesh.applyTexture(sl::MESH_TEXTURE_RGB);
        ROS_INFO("Mesh has been textured...");
    }

    // Save as an OBJ file
    std::string saveName = "./mesh_gen.obj";
    bool t = wholeMesh.save(saveName.c_str());
    if (t) ROS_INFO("Mesh has been saved under %s", saveName.c_str());
    else ROS_WARN("Failed to save the mesh under %s", saveName.c_str());
}

// Update the mesh and draw image and wireframe using OpenGL
void ZedSpatialMapper::update()
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

zed_wrapper::Matrix4f toMatrixMsg(sl::Transform &transform)
{
	zed_wrapper::Matrix4f matrix;
	matrix.m11 = transform(0,0);
	matrix.m12 = transform(0,1);
	matrix.m13 = transform(0,2);
	matrix.m14 = transform(0,3);
	matrix.m21 = transform(1,0);
	matrix.m22 = transform(1,1);
	matrix.m23 = transform(1,2);
	matrix.m24 = transform(1,3);
	matrix.m31 = transform(2,0);
	matrix.m32 = transform(2,1);
	matrix.m33 = transform(2,2);
	matrix.m34 = transform(2,3);
	matrix.m41 = transform(3,0);
	matrix.m42 = transform(3,1);
	matrix.m43 = transform(3,2);
	matrix.m44 = transform(3,3);
	return matrix;
}

void ZedSpatialMapper::publish(ros::Publisher &pub_mesh)
{
	if (is_mapping)
	{
		// Update the pose and projection
		mesh_msg.perspective = toMatrixMsg(perspective);
		mesh_msg.pose = toMatrixMsg(pose.pose_data);

		pub_mesh.publish(mesh_msg);
	}
}

void ZedSpatialMapper::updateMsg()
{
	for (int c = 0; c < mesh.chunks.size(); c++)
	{
		// If the mesh msg is not as large as the real mesh, increase it
		if (mesh_msg.chunks.size() < mesh.chunks.size()) mesh_msg.chunks.emplace_back();

		// If this mesh chunk has been updated since the last update to the msg
		if (mesh.chunks[c].has_been_updated)
		{
			zed_wrapper::Chunk chunk_msg;
			// Get the new vertices
			for (auto &vertex : mesh.chunks[c].vertices)
			{
				zed_wrapper::Vertex vertex_msg;
				vertex_msg.x = vertex[0];
				vertex_msg.y = vertex[1];
				vertex_msg.z = vertex[2];
				chunk_msg.vertices.push_back(vertex_msg);
			}
			// Get the new triangles
			for (auto &triangle : mesh.chunks[c].triangles)
			{
				zed_wrapper::Triangle triangle_msg;
				triangle_msg.x = triangle[0];
				triangle_msg.y = triangle[1];
				triangle_msg.z = triangle[2];
				chunk_msg.triangles.push_back(triangle_msg);
			}
			// Get the new normals
			for (auto &normal : mesh.chunks[c].normals)
			{
				zed_wrapper::Normal normal_msg;
				normal_msg.x = normal[0];
				normal_msg.y = normal[1];
				normal_msg.z = normal[2];
				chunk_msg.normals.push_back(normal_msg);
			}
			// Get the new UVs
			for (auto &uv : mesh.chunks[c].uv)
			{
				zed_wrapper::UV uv_msg;
				uv_msg.u = uv[0];
				uv_msg.v = uv[1];
				chunk_msg.uv.push_back(uv_msg);
			}
			// Insert the updated chunk into the same position as the sl::Mesh
			mesh_msg.chunks[c] = chunk_msg;
		}
	}
}

sl::Transform ZedSpatialMapper::createPerspective()
{
    // Create projection matrix. Use this matrix in combination with the Pose
    // (on REFERENCE_FRAME_WORLD) to project the mesh on the 2D Image.
    sl::Transform projection;
    sl::CameraParameters camLeft = zed->getCameraInformation().calibration_parameters.left_cam;
    projection(0, 0) = 1.0f / tanf(camLeft.h_fov * M_PI / 180.f * 0.5f);
    projection(1, 1) = 1.0f / tanf(camLeft.v_fov * M_PI / 180.f * 0.5f);
    float znear = 0.001f;
    float zfar = 100.f;
    projection(2, 2) = (zfar + znear) / (zfar - znear);
    projection(2, 3) = (2.f * zfar * znear) / (zfar - znear);
    projection(3, 2) = 1.f;
    projection(0, 2) = (camLeft.image_size.width - 2.f * camLeft.cx) / camLeft.image_size.width;
    projection(1, 2) = (-1.f * camLeft.image_size.height + 2.f * camLeft.cy) / camLeft.image_size.height;
    projection(3, 3) = 0.f;
    return projection;
}