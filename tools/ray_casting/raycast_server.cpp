#include "ros/ros.h"
#include "RaycastMesh.h"
#include "wavefront.h"

RmReal raycast()
{
    RmReal hitLocation[3];
    RmReal normal[3];
    RmReal hitDistance;
    bool hit = rm->raycast(from,to,hitLocation,normal,&hitDistance);
    return hitLocation;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "raycast_server");
    ros::NodeHandle n;
    if ( argc != 2 )
    {
        printf("Usage: RaycastMesh <wavefront.obj>\r\n");
        return 0;
    }
    else
    {
        printf("Loading wavefront obj file '%s'\r\n", argv[1] );
        WavefrontObj obj;
        unsigned int tcount = obj.loadObj(argv[1],false);
        if ( tcount )
        {
            printf("Creating Raycast Mesh. Building AABB\r\n");
            RaycastMesh *rm = createRaycastMesh(obj.mVertexCount,obj.mVertices,obj.mTriCount,(const RmUint32 *)obj.mIndices);
            if ( !rm )
            {
                printf("Error creating raycast mesh");
                return 0;
            }
        }
    }
    ros::ServiceServer service = n.advertiseService("add_two_ints", raycast);
    ROS_INFO("Ready to add two ints.");
    ros::spin();
    return 0;
}