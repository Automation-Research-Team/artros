/*!
  \file		textured_mesh_display.cpp
  \author	Toshio Ueshiba
*/
#include <OGRE/OgreMaterialManager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "textured_mesh_display.h"

namespace rviz
{
/************************************************************************
*  static functions							*
************************************************************************/
static Ogre::Vector3
fromMsg(const geometry_msgs::Point& p)
{
    return {float(p.x), float(p.y), float(p.z)};
}

/************************************************************************
*  class TexturedMeshDisplay						*
************************************************************************/
/*
 *  public member functions
 */
TexturedMeshDisplay::TexturedMeshDisplay()
    :Display(),
     image_topic_property_(new RosTopicProperty(
			       "Image Topic", "",
			       QString::fromStdString(
				   ros::message_traits::datatype<image_t>()),
			       "Image topic to subscribe to.",
			       this, SLOT(updateDisplayImages()))),
     mesh_topic_property_(new RosTopicProperty(
			      "Mesh Topic", "",
			      QString::fromStdString(
				  ros::message_traits::datatype<mesh_t>()),
			      "Mesh topic to subscribe to.",
			      this, SLOT(updateDisplayImages()))),
     texture_(new ROSImageTexture())
{
}

TexturedMeshDisplay::~TexturedMeshDisplay()
{
    unsubscribe();
}

/*
 *  public mamber functions : Overrides from Display
 */
void
TexturedMeshDisplay::onInitialize()
{
    Display::onInitialize();
}

void
TexturedMeshDisplay::onEnable()
{
    subscribe();
}

void
TexturedMeshDisplay::onDisable()
{
    unsubscribe();
}

void
TexturedMeshDisplay::update(float wall_dt, float ros_dt)
{
    try
    {
	if (cur_mesh_ && cur_image_)
	{
	    createTexture();		// Create texture_ from cur_image_
	    createMesh();		// Create mesh_node_ from cur_mesh_
	    updateMeshProperties();

	    if (texture_ &&
		!image_topic_property_->getTopic().isEmpty() &&
		!mesh_topic_property_->getTopic().isEmpty())
	    {
		texture_->update();
		updateCamera();
	    }
	}

	setStatus(StatusProperty::Ok, "Display Image", "ok");
    }
    catch (const std::exception& e)
    {
	setStatus(StatusProperty::Error, "Display Image", e.what());
    }
}

void
TexturedMeshDisplay::reset()
{
    Display::reset();
    texture_->clear();
    context_->queueRender();
    setStatus(StatusProperty::Warn, "Image", "No Image received");
}

/*
 *  protected member functions
 */
void
TexturedMeshDisplay::subscribe()
{
    if (!isEnabled())
	return;

    if (!image_topic_property_->getTopic().isEmpty())
    {
	try
	{
	    image_sub_ = nh_.subscribe(image_topic_property_->getTopicStd(), 1,
				       &TexturedMeshDisplay::updateImage,
				       this);
	    setStatus(StatusProperty::Ok, "Display Images Topic", "ok");
	}
	catch (ros::Exception& e)
	{
	    setStatus(StatusProperty::Error, "Display Images Topic",
		      QString("Error subscribing: ") + e.what());
	}
    }

    if (!mesh_topic_property_->getTopic().isEmpty())
    {
	try
	{
	    mesh_sub_ = nh_.subscribe(mesh_topic_property_->getTopicStd(), 1,
				      &TexturedMeshDisplay::updateMesh, this);
	    setStatus(StatusProperty::Ok, "Mesh Topic", "ok");
	}
	catch (ros::Exception& e)
	{
	    setStatus(StatusProperty::Error, "Mesh Topic",
		      QString("Error subscribing: ") + e.what());
	}
    }
}

void
TexturedMeshDisplay::unsubscribe()
{
    image_sub_.shutdown();
    mesh_sub_.shutdown();
}

/*
 *  private member functions
 */
void
TexturedMeshDisplay::updateImage(const image_cp& image)
{
    std::lock_guard<std::mutex>	lock(image_mutex_);

    cur_image_ = image;
}

void
TexturedMeshDisplay::updateMesh(const mesh_cp& mesh)
{
    std::lock_guard<std::mutex>	lock(mesh_mutex_);

    cur_mesh_ = mesh;
}

void
TexturedMeshDisplay::createTexture()
{
    std::lock_guard<std::mutex>	lock(image_mutex_);

    const auto	img = cv_bridge::toCvCopy(*cur_image_,
					  sensor_msgs::image_encodings::RGBA8);
    texture_->addMessage(img->toImageMsg());
}

void
TexturedMeshDisplay::createMesh()
{
  // Create mesh node, mesh material and manual object if not created yet.
    if (!mesh_node_)
    {
	mesh_node_.reset(scene_node_->createChildSceneNode());
	manual_object_.reset(context_->getSceneManager()
				     ->createManualObject("MeshObject"));
	mesh_node_->attachObject(manual_object_.get());

	const Ogre::String resource_group_name = "MeshNode";
	auto&&		   rg_mgr = Ogre::ResourceGroupManager::getSingleton();
	if (!rg_mgr.resourceGroupExists(resource_group_name))
	{
	    rg_mgr.createResourceGroup(resource_group_name);

	    mesh_material_ = Ogre::MaterialManager::getSingleton().create(
				resource_group_name + "MeshMaterial",
				resource_group_name);
	}
    }

    std::lock_guard<std::mutex>	lock(mesh_mutex_);

  // Get a transform from mesh frame to RViz display frame.
    Ogre::Vector3	position;
    Ogre::Quaternion	orientation;
    if (!context_->getFrameManager()->getTransform(cur_mesh_->header.frame_id,
						   ros::Time(0),
						   position, orientation))
	throw std::runtime_error("Error transforming from mesh frame["
				 + cur_mesh_->header.frame_id
				 + "] to current RViz frame");
    Ogre::Matrix4	transform;
    transform.makeTransform(position, Ogre::Vector3(1.0, 1.0, 1.0),
			    orientation);

  // Compute normals for all vertices.
    std::vector<Ogre::Vector3>	normals(cur_mesh_->mesh.vertices.size());
    for (const auto& triangle : cur_mesh_->mesh.triangles)
    {
	const auto	idx0 = triangle.vertex_indices[0];
	const auto	idx1 = triangle.vertex_indices[1];
	const auto	idx2 = triangle.vertex_indices[2];
	const auto	vtx0 = fromMsg(cur_mesh_->mesh.vertices[idx0]);
	const auto	vtx1 = fromMsg(cur_mesh_->mesh.vertices[idx1]);
	const auto	vtx2 = fromMsg(cur_mesh_->mesh.vertices[idx2]);
	auto		normal = (vtx1 - vtx0).crossProduct(vtx2 - vtx1);
	normal.normalise();
	normals[idx0] = normal;
	normals[idx1] = normal;
	normals[idx2] = normal;
    }

  // Add positions, normals and texture coordinates to the manual object.
    if (manual_object_->getCurrentVertexCount()
	== cur_mesh_->mesh.vertices.size())
    {
	manual_object_->beginUpdate(0);
    }
    else
    {
	manual_object_->clear();
	manual_object_->estimateVertexCount(cur_mesh_->mesh.vertices.size());
	manual_object_->begin(mesh_material_->getName(),
			      Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    for (size_t i = 0; i < cur_mesh_->mesh.vertices.size(); ++i)
    {
	const auto&	vertex = cur_mesh_->mesh.vertices[i];
	manual_object_->position(transform.transformAffine(fromMsg(vertex)));
	manual_object_->normal(orientation * normals[i]);
	manual_object_->textureCoord(cur_mesh_->u[i], cur_mesh_->v[i]);
    }

    for (const auto& triangle : cur_mesh_->mesh.triangles)
    {
	manual_object_->triangle(triangle.vertex_indices[0],
				 triangle.vertex_indices[1],
				 triangle.vertex_indices[2]);
    }

    manual_object_->end();
}

void
TexturedMeshDisplay::updateMeshProperties()
{
    const auto	pass = mesh_material_->getTechnique(0)->getPass(0);
    pass->setSelfIllumination(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
    pass->setDiffuse(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    pass->setAmbient(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
    pass->setSpecular(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    pass->setShininess(64.0f);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    context_->queueRender();
}

void
TexturedMeshDisplay::updateCamera()
{
    const auto	pass = mesh_material_->getTechnique(0)->createPass();
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);

    const auto	tex_state = pass->createTextureUnitState();
    tex_state->setTextureName(texture_->getTexture()->getName());
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR,
				   Ogre::FO_NONE);
    tex_state->setColourOperation(Ogre::LBO_REPLACE);  // don't accept addition
}

/*
 *  private member functions: Q_SLOTS
 */
void
TexturedMeshDisplay::updateDisplayImages()
{
    unsubscribe();
    subscribe();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::TexturedMeshDisplay, rviz::Display)
