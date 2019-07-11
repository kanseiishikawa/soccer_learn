#include <vector>       // ヘッダファイルインクルード
#include <iostream>
#include <string>

#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/angle_deg.h>
#include <rcsc/geom/rect_2d.h>
#include <rcsc/geom/size_2d.h>
#include <rcsc/geom/ray_2d.h>
#include <rcsc/geom/circle_2d.h>

#ifndef THROUGH_GENERATE_H
#define THROUGH_GENERATE_H

class through_generate
{
 public:
    bool createPassCommon( const rcsc::Vector2D & receive_point,
                           const int min_step,
                           const int max_step,
                           const int step,//maxstepからmindtepの間
                           const int kicker_unum,
                           const int self_unum,
                           const double & min_first_ball_speed,
                           const double & max_first_ball_speed,
                           const double & min_receive_ball_speed,
                           const double & max_receive_ball_speed,
                           const double & ball_move_dist,
                           const bool kickable,
                           const bool play_on,
                           const bool penalty_kick_mode,
                           const char * description );

    int predictOpponentReachStep( const rcsc::Vector2D & first_ball_pos,
                                  const rcsc::Vector2D & first_ball_vel,
                                  const rcsc::Vector2D & receive_point,
                                  const rcsc::Vector2D & opponent_pos,
                                  const rcsc::Vector2D & opponent_vel,
                                  const rcsc::AngleDeg & ball_move_angle,
                                  const double opp_kick_ablearea,
                                  const double offside_line_x,
                                  double opp_real_max_speed,
                                  bool goalie,
                                  const int max_cycle,
                                  int opp_seen_pos_count,
                                  int opp_pos_count,
                                  const char M_pass_type );

    double estimate_virtual_dash_distance( int player_seen_pos_count,
                                           int player_pos_count,
                                           double player_real_max_speed );



};
#endif
