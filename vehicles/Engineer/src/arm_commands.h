
#include "geometry/geometry.h"

class Command{
    virtual bool is_finished();
};

class MoveToPositionCommand : public Command{
    public: 
        static const int TYPE = 1;
        Vector3d end_position;
        Vector3d current_position;
        bool is_finished(){
            return (current_position-end_position).length() < 0.02;
        }
        void updateCurrentPosition(Vector3d curr_position){
            current_position = curr_position;
        }
};


