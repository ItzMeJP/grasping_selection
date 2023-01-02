#include "core.h"

using namespace grasping_selection;
using namespace std;

int main()
{
    GraspingSelection _server;
    Configuration c;
    c.log_folder_path = "/home/joaopedro/";


    if( _server.setupConfiguration(c) )
        cout << "Success" << endl;


    RequestInput in;

    in.operation_mode = GraspingSelection::OPERATION_MODE::STANDALONE_RUN;
    in.detected_object_name = "object";
    in.detected_object_tf_name = "object";

    if( _server.requestSelection(in) )
        cout << "Success" << endl;

    _server.buildTheLogFile();


    return 0;

}