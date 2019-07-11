#include<iostream>
#include<cmath>

#include "soccer_lib.h"

using namespace rcsc;

int
soccer_lib::estimate_min_reach_cycle( const Vector2D & player_pos,
                                      const Vector2D & target_first_point,
                                      const AngleDeg & target_move_angle )
{
    rcsc::Vector2D target_to_player = ( player_pos - target_first_point ).rotatedVector( -target_move_angle );
    return ( target_to_player.x < -1.0
             ? -1
             : std::max( 1, static_cast< int >( std::floor( target_to_player.absY() / player_speed_max ) ) ) );
}

double
soccer_lib::calc_first_term_geom_series( const double & sum,
                                         const double & r,
                                         const int len )
{
    return sum * ( 1.0 - r ) / ( 1.0 - std::pow( r, len ) );
}

int
soccer_lib::predict_kick_count( const double & first_ball_speed,
                                double self_kick_rate,
                                AngleDeg ball_move_angle,
                                Vector2D ball_vel,
                                int kicker_unum,
                                int self_unum,
                                bool kickable,
                                bool play_on,
                                bool penalty_kick_mode )
{
    if ( ! play_on
         && ! penalty_kick_mode )
    {
        return 1;
    }

    if ( kicker_unum == self_unum
         && kickable )
    {
        Vector2D max_vel = calc_max_velocity( ball_move_angle,
                                              self_kick_rate,
                                              ball_vel );
        
        if ( max_vel.r2() >= std::pow( first_ball_speed, 2 ) )
        {
            return 1;
        }
    }

    if ( first_ball_speed > 2.5 )
    {
        return 3;
    }
    else if ( first_ball_speed > 1.5 )
    {
        return 2;
    }

    return 1;
}

Vector2D
soccer_lib::calc_max_velocity( const AngleDeg & target_angle,
                              const double & krate,
                              const Vector2D & ball_vel )
{
    const double ball_speed_max2 = std::pow( ball_speed_max, 2 );
    const double max_accel
        = std::min( max_power * krate,
                    ball_accel_max );

    Ray2D desired_ray( Vector2D( 0.0, 0.0 ), target_angle );
    Circle2D next_reachable_circle( ball_vel, max_accel );

    Vector2D vel1, vel2;
    int num = next_reachable_circle.intersection( desired_ray, &vel1, &vel2 );

    if ( num == 0 )
    {
        return Vector2D( 0.0, 0.0 );
    }

    if ( num == 1 )
    {
        if ( vel1.r2() > ball_speed_max2 )
        {
            // next inertia ball point is within reachable circle.
            if ( next_reachable_circle.contains( Vector2D( 0.0, 0.0 ) ) )
            {
                // can adjust angle at least
                vel1.setLength( ball_speed_max );
            }
            else
            {
                // failed
                vel1.assign( 0.0, 0.0 );
            }
        }
        return vel1;
    }

    //
    // num == 2
    //   ball reachable circle does not contain the current ball pos.

    double length1 = vel1.r2();
    double length2 = vel2.r2();

    if ( length1 < length2 )
    {
        std::swap( vel1, vel2 );
        std::swap( length1, length2 );
    }

    if ( length1 > ball_speed_max2 )
    {
        if ( length2 > ball_speed_max2 )
        {
            // failed
            vel1.assign( 0.0, 0.0 );
        }
        else
        {
            vel1.setLength( ball_speed_max );
        }
    }

    return vel1;
}

Vector2D
soccer_lib::inertia_n_step_point( Vector2D first_ball_pos,
                                  Vector2D first_ball_vel,
                                  int cycle,
                                  double ball_decay )
{
    Vector2D next_ball_pos = first_ball_pos;
    Vector2D next_ball_vel = first_ball_vel;
    
    for( int i = 0; i < cycle ; i++ )
    {
        next_ball_pos += next_ball_vel;
        next_ball_vel = next_ball_vel * ball_decay;
    }

    return next_ball_pos;
}

Vector2D
soccer_lib::player_interia_pos( Vector2D player_pos,
                                Vector2D player_vel,
                                int cycle )
{
    Vector2D next_player_pos = player_pos;
    Vector2D next_player_vel = player_vel;
    
    for( int i = 0; i < cycle; i++ )
    {
        next_player_pos += player_vel;
        next_player_vel = next_player_vel * player_decay;
    }

    return next_player_pos;
}



