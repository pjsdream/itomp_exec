#include <ros/ros.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

bool defaultCollisionFunction(fcl::CollisionObject* obj1, fcl::CollisionObject* obj2, void* cdata)
{
    fcl::Vec3f t1 = obj1->getTranslation();
    fcl::Vec3f t2 = obj2->getTranslation();
    
    printf("callback called (%f, %f, %f), (%f, %f, %f)\n", t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]);
    
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_fcl");
    
    fcl::Transform3f tf1;
    tf1.setTranslation(fcl::Vec3f(0, 0, 0));
    boost::shared_ptr<fcl::Sphere> geom1(new fcl::Sphere(1.0));
    fcl::CollisionObject* obj1 = new fcl::CollisionObject(geom1, tf1);
    
    fcl::Transform3f tf2;
    tf2.setTranslation(fcl::Vec3f(1.5, 0, 0));
    boost::shared_ptr<fcl::Sphere> geom2(new fcl::Sphere(1.0));
    fcl::CollisionObject* obj2 = new fcl::CollisionObject(geom2, tf2);
    
    fcl::Transform3f tf3;
    tf3.setTranslation(fcl::Vec3f(0, 1, 0));
    boost::shared_ptr<fcl::Sphere> geom3(new fcl::Sphere(1.0));
    fcl::CollisionObject* obj3 = new fcl::CollisionObject(geom3, tf3);
    
    fcl::Transform3f tf4;
    tf4.setTranslation(fcl::Vec3f(1.5, 1, 0));
    boost::shared_ptr<fcl::Sphere> geom4(new fcl::Sphere(1.0));
    fcl::CollisionObject* obj4 = new fcl::CollisionObject(geom4, tf4);
    
    fcl::BroadPhaseCollisionManager* manager1 = new fcl::DynamicAABBTreeCollisionManager();
    manager1->registerObject(obj1);
    manager1->registerObject(obj2);
    manager1->setup();
    
    fcl::BroadPhaseCollisionManager* manager2 = new fcl::DynamicAABBTreeCollisionManager();
    manager2->registerObject(obj3);
    manager2->registerObject(obj4);
    manager2->setup();
    
    fcl::CollisionRequest collision_request;
    fcl::CollisionResult collision_result;
    
    collision_request.enable_contact = true;
    collision_request.num_max_contacts = 1000;
    
    /*
    fcl::collide(obj1, obj2, collision_request, collision_result);
    
    for (int i=0; i<collision_result.numContacts(); i++)
    {
        fcl::Contact contact = collision_result.getContact(i);
        const double depth = contact.penetration_depth;
        const fcl::Vec3f pos = contact.pos;
        const fcl::Vec3f normal = contact.normal;
        
        printf("depth = %lf, pos = (%lf, %lf, %lf), normal = (%lf, %lf, %lf)\n", depth, pos[0], pos[1], pos[2], normal[0], normal[1], normal[2]);
        
        // it seems that
        // 1. pos is the center of two points involved in penetration depth
        // 2. normal is the direction to which obj1 should move to avoid collision
    }
    */
    
    printf("test 1\n");
    manager2->collide(manager1, 0, &defaultCollisionFunction);
    
    printf("test 2\n");
    obj4->setTranslation(fcl::Vec3f(25, 25, 0));
    obj4->computeAABB();
    manager2->update();
    manager2->collide(manager1, 0, &defaultCollisionFunction);
    
    // to update collision object transformation:
    // 1. set transformation to a new one
    // 2. CollisionObject::computeAABB()
    // 3. DynamicAABBTreeCollisionManager::update()
    
    return 0;
}
