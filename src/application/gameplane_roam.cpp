/**
 * @file gameplane_roam.cpp
 * @brief GameplaneControl class
 *
 * @author Zeyang Li (zeyangl)
 * @author Eric Butler (edbutler)
 * @author Minwong Ji (minwongj) 
 * @description this file is copied and modified from camera_roam.cpp
 */

#include "application/gameplane_roam.hpp"

namespace _462 {

static const real_t DirectionTable[] = { 0.0, 1.0, -1.0 };
static const real_t TranslationSpeed = 15.0;
static const real_t RotationSpeed = 0.02;

GameplaneControl::GameplaneControl()
{
    direction[0] = DZERO;
    direction[1] = DZERO;
    direction[2] = DZERO;
    rotation = RNONE;
    mouseMovementX = 0.0;
    mouseMovementY = 0.0;
    previousMouseX = 0.0;
    previousMouseY = 0.0;
    is_first_time = true;
    gameplane = NULL;
}

GameplaneControl::~GameplaneControl()
{

}

void GameplaneControl::set_dir( bool pressed, int index, Direction newdir )
{
    // fi pressed set, otherwise only undo it if other direction is not the one being pressed
    if ( pressed )
        direction[index] = newdir;
    else if ( direction[index] == newdir )
        direction[index] = DZERO;
}

void GameplaneControl::handle_event( const Application* app, const SDL_Event& event )
{
    int width, height;
    app->get_dimension( &width, &height );
    int newidx = -1;
    Direction newdir = DZERO;

    switch ( event.type )
    {
    case SDL_KEYDOWN:
    case SDL_KEYUP:
        switch( event.key.keysym.sym )
        {
        case SDLK_i:
            newidx = 2;
            newdir = DNEG;
            break;
        case SDLK_k:
            newidx = 2;
            newdir = DPOS;
            break;
        case SDLK_j:
            newidx = 0;
            newdir = DNEG;
            break;
        case SDLK_l:
            newidx = 0;
            newdir = DPOS;
            break;
        case SDLK_u:
            newidx = 1;
            newdir = DNEG;
            break;
        case SDLK_o:
            newidx = 1;
            newdir = DPOS;
            break;
        default:
            newidx = -1;
            break;
        }

        if ( newidx != -1 ) {
            set_dir( event.key.state == SDL_PRESSED, newidx, newdir );
        }
        break;

    case SDL_MOUSEBUTTONDOWN:
        // enable rotation
        if ( event.button.button == SDL_BUTTON_LEFT )
            rotation = RPITCHYAW;
        else if ( event.button.button == SDL_BUTTON_MIDDLE )
            rotation = RROLL;
        break;

    case SDL_MOUSEBUTTONUP:
        // disable rotation
        if ( event.button.button == SDL_BUTTON_LEFT && rotation == RPITCHYAW )
            rotation = RNONE;
        else if ( event.button.button == SDL_BUTTON_MIDDLE && rotation == RROLL )
            rotation = RNONE;
        break;

    case SDL_MOUSEMOTION:
/*
        // initialize
        if (is_first_time) {
            previousMouseX = event.motion.xrel;
            previousMouseY = event.motion.yrel;
            is_first_time = false;
        }
        
        //newidx = 2;
        //newdir = DPOS;

        if (previousMouseX - event.motion.xrel != 0)
            mouseMovementX = previousMouseX - event.motion.xrel;
        if (previousMouseY - event.motion.yrel != 0)
            mouseMovementY = previousMouseY - event.motion.yrel;
       
        previousMouseX = event.motion.xrel;
        previousMouseY = event.motion.yrel;


        if ( newidx != -1 ) {
            set_dir(true, newidx, newdir );
        }
*/
    default:
        break;
    }
    
}

void GameplaneControl::update( real_t dt )
{
    // update the position based on keys
    // no need to update based on mouse, that's done in the event handling
    real_t dist = TranslationSpeed * dt;
    Vector3 displacement(
        DirectionTable[direction[0]],
        DirectionTable[direction[1]],
        DirectionTable[direction[2]]
    );
    gameplane->translate( displacement * dist );
    //gameplane->translate( Vector3(1, 0, 0) * -mouseMovementX * dt);
    //gameplane->translate( Vector3(0, 0, 1) * -mouseMovementY * dt);
}

}
