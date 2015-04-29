/*
 * @file gameplane_roam.cpp
 * @brief GameplaneControl class
 *
 * @author Zeyang Li (zeyangl)
 * @author Minwong Ji (minwongj) 
 * @description this file is copied and modified from camera_roam.hpp
 */

#ifndef _462_APPLICATION_GAMEPLANEROAM_HPP_
#define _462_APPLICATION_GAMEPLANEROAM_HPP_

#include "application/application.hpp"
#include "p5/body.hpp"

namespace _462 {

class GameplaneControl
{
public:

    GameplaneControl();
    ~GameplaneControl();

    void update( real_t dt );
    void handle_event( const Application* app, const SDL_Event& event );

    // the plane object of this control
    Body* gameplane;

private:

    enum Direction { DZERO=0, DPOS=1, DNEG=2 };
    enum Rotation { RNONE, RPITCHYAW, RROLL };

    void set_dir( bool pressed, int index, Direction newdir );

    // current directions in the local camera axes, (x, y, z)
    Direction direction[3];
    // the current rotation
    Rotation rotation;

    real_t mouseMovementX;
    real_t mouseMovementY;

    real_t previousMouseX;
    real_t previousMouseY;

    bool is_first_time;

};

} /* _462 */

#endif /* _462_APPLICATION_CAMERAROAM_HPP_ */

