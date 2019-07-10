#include <vector>       // ヘッダファイルインクルード
#include <iostream>
#include <string>

#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/angle_deg.h>
#include <rcsc/geom/rect_2d.h>
#include <rcsc/geom/size_2d.h>
#include <rcsc/geom/ray_2d.h>
#include <rcsc/geom/circle_2d.h>

#ifndef SOCCER_LIB_H
#define SOCCER_LIB_H

class soccer_lib
{
private:
    int a;
public:
    const double ball_decay = 0.94;
    const double ball_speed_max = 3.0;
    const double max_power = 100;
    const double ball_accel_max = 2.7;
    const double their_penalty_area_x = 35.0;
    const double penalty_area_length = 16.5;
    const double penalty_area_width = 40.0;
    const double player_speed_max = 1.05;
    const double catch_ablearea = 1.2;
    const double player_decay = 0.4;

    int estimate_min_reach_cycle( const rcsc::Vector2D & player_pos,
                                  const rcsc::Vector2D & target_first_point,
                                  const rcsc::AngleDeg & target_move_angle );

    int predict_kick_count( const double & first_ball_speed,
                            double self_kick_rate,
                            rcsc::AngleDeg ball_move_angle,
                            rcsc::Vector2D ball_vel,
                            int kicker_unum,
                            int self_unum,
                            bool kickable,
                            bool play_on,
                            bool penalty_kick_mode );


    double calc_first_term_geom_series( const double & sum,
                                        const double & r,
                                        const int len );

    rcsc::Vector2D inertia_n_step_point( rcsc::Vector2D first_ball_pos,
                                         rcsc::Vector2D first_ball_vel,
                                         int cycle,
                                         double ball_decay );

    rcsc::Vector2D player_interia_pos( rcsc::Vector2D player_pos,
                                       rcsc::Vector2D player_vel,
                                       int cycle );

    rcsc::Vector2D
    calc_max_velocity( const rcsc::AngleDeg & target_angle,
                       const double & krate,
                       const rcsc::Vector2D & ball_vel );


    
};
#endif
