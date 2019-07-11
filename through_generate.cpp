#include<iostream>
#include<cmath>
#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/angle_deg.h>
#include <rcsc/geom/rect_2d.h>
#include <rcsc/geom/size_2d.h>

#include "through_generate.h"
#include "soccer_lib.h"

using namespace rcsc;

/*
bool
through_generate::createPassCommon( const rcsc::Vector2D & receive_point,
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
                                    const char * description )
{

    int success_count = 0;
#ifdef DEBUG_PRINT_SUCCESS_PASS
    std::vector< int > success_counts;
    success_counts.reserve( max_step - min_step + 1 );
#endif

    double first_ball_speed = calc_first_term_geom_series( ball_move_dist,
                                                           ball_decay,
                                                           step );


    if ( first_ball_speed < min_first_ball_speed )
    {
        return false;
    }

    if ( max_first_ball_speed < first_ball_speed )
    {
        return false;
    }

    double receive_ball_speed = first_ball_speed * std::pow( ball_decay, step );
    if ( receive_ball_speed < min_receive_ball_speed )
    {
        return false;
    }

    if ( max_receive_ball_speed < receive_ball_speed )
    {
        return false;
    }




    int kick_count = predict_kick_count( first_ball_speed, kicker_unum, self_unum, kickable, play_on, penalty_kick_mode );

    //const AbstractPlayerObject * opponent = static_cast< const AbstractPlayerObject * >( 0 );
    int o_step = predictOpponentsReachStep( wm,
                                            M_first_point,
                                            first_ball_speed,
                                            ball_move_angle,
                                            receive_point,
                                            step + ( kick_count - 1 ) + 5,
                                            &opponent );

    bool failed = false;
    if ( M_pass_type == 'T' )
    {
        if ( o_step <= step )
        {
            failed = true;
        }

        if ( receive_point.x > 30.0
             && step >= 15
             && ( ! opponent
                  || ! opponent->goalie() )
             && o_step >= step ) // Magic Number
        {
            AngleDeg receive_move_angle = ( receive_point - receiver.pos_ ).th();
            if ( ( receiver.player_->body() - receive_move_angle ).abs() < 15.0 )
            {
                failed = false;
            }
        }
    }
    else
    {
        if ( o_step <= step + ( kick_count - 1 ) )
        {
            failed = true;
        }
    }

    if ( failed )
    {
        return false;
    }

    //success
    return true;
}


int
through_generate::predictOpponentsReachStep( const WorldModel & wm,
                                             const Vector2D & first_ball_pos,
                                             const double & first_ball_speed,
                                             const AngleDeg & ball_move_angle,
                                             const Vector2D & receive_point,
                                             const int max_cycle,
                                             const AbstractPlayerObject ** opponent )
{
    const Vector2D first_ball_vel = Vector2D::polar2vector( first_ball_speed, ball_move_angle );

    double bonus_dist = -10000.0;
    int min_step = 1000;
    const AbstractPlayerObject * fastest_opponent = static_cast< AbstractPlayerObject * >( 0 );

    for ( OpponentCont::const_iterator o = M_opponents.begin();
          o != M_opponents.end();
          ++o )
    {
        int step = predictOpponentReachStep( wm,
                                             *o,
                                             first_ball_pos,
                                             first_ball_vel,
                                             ball_move_angle,
                                             receive_point,
                                             std::min( max_cycle, min_step ) );
        if ( step < min_step
             || ( step == min_step
                  && o->bonus_distance_ > bonus_dist ) )
        {
            bonus_dist = o->bonus_distance_;
            min_step = step;
            fastest_opponent = o->player_;
        }
    }

    if ( opponent )
    {
        *opponent = fastest_opponent;
    }
    return min_step;
}
*/

int
through_generate::predictOpponentReachStep( const Vector2D & first_ball_pos,
                                            const Vector2D & first_ball_vel,
                                            const Vector2D & receive_point,
                                            const Vector2D & opponent_pos,
                                            const Vector2D & opponent_vel,
                                            const AngleDeg & ball_move_angle,
                                            const double opp_kick_ablearea,
                                            const double offside_line_x,
                                            double opp_real_max_speed,
                                            bool goalie,
                                            const int max_cycle,
                                            int opp_seen_pos_count,
                                            int opp_pos_count,
                                            const char M_pass_type )
{
    static const Rect2D penalty_area( Vector2D( soccer_lib().penalty_area_length,
                                                -soccer_lib().penalty_area_width / 2 ),
                                      Size2D( soccer_lib().penalty_area_length,
                                              soccer_lib().penalty_area_width ) );
    static const double CONTROL_AREA_BUF = 0.15;

    const int min_cycle = soccer_lib().estimate_min_reach_cycle( opponent_pos,
                                                                 first_ball_pos,
                                                                 ball_move_angle );
    if ( min_cycle < 0 )
    {
        return 1000;
    }

    for ( int cycle = std::max( 1, min_cycle ); cycle <= max_cycle; ++cycle )
    {
        const Vector2D ball_pos = soccer_lib().inertia_n_step_point( first_ball_pos,
                                                                     first_ball_vel,
                                                                     cycle,
                                                                     soccer_lib().ball_decay );
        const double control_area = ( goalie
                                      && penalty_area.contains( ball_pos )
                                      ? soccer_lib().catch_ablearea
                                      : opp_kick_ablearea );

        const Vector2D inertia_pos = soccer_lib().player_interia_pos( opponent_pos, opponent_vel, cycle );
        const double target_dist = inertia_pos.dist( ball_pos );

        double dash_dist = target_dist;

        if ( M_pass_type == 'T'
             && first_ball_vel.x > 2.0
             && ( receive_point.x > offside_line_x
                  || receive_point.x > 30.0 ) )
        {
        }
        else
        {
            dash_dist -= estimate_virtual_dash_distance( opp_seen_pos_count, opp_pos_count, opp_real_max_speed);
        }

        if ( dash_dist - control_area - CONTROL_AREA_BUF < 0.001 )
        {
            return cycle;
        }

        //if ( cycle > 1 )
        {
            if ( M_pass_type == 'T'
                 && first_ball_vel.x > 2.0
                 && ( receive_point.x > offside_line_x
                      || receive_point.x > 30.0 ) )
            {
                //dash_dist -= control_area * 0.5;
                //dash_dist -= control_area * 0.8;
                dash_dist -= control_area;
            }
            else
            {
                if ( receive_point.x < 25.0 )
                {
                    dash_dist -= control_area;
                    dash_dist -= 0.5;
                }
                else
                {
                    //dash_dist -= control_area * 0.8;
                    dash_dist -= control_area;
                    dash_dist -= 0.2;
                }
            }
        }

        if ( dash_dist > ptype->realSpeedMax()
             * ( cycle + std::min( opp_pos_count, 5 ) ) )
        {
            continue;
        }

        //
        // dash
        //

        int n_dash = ptype->cyclesToReachDistance( dash_dist );

        if ( n_dash > cycle + opp_pos_count )
        {
            continue;
        }

        //
        // turn
        //
        int n_turn = ( opponent.player_->bodyCount() > 1
                       ? 0
                       : FieldAnalyzer::predict_player_turn_cycle( ptype,
                                                                   opponent.player_->body(),
                                                                   opponent.speed_,
                                                                   target_dist,
                                                                   ( ball_pos - inertia_pos ).th(),
                                                                   control_area,
                                                                   true ));

        int n_step = n_turn + n_dash;

        int bonus_step = 0;
        if ( opponent.player_->isTackling() )
        {
            bonus_step = -5; // Magic Number
        }

        // if ( receive_point.x < 0.0 )
        // {
        //     bonus_step += 1;
        // }

        if ( n_step - bonus_step <= cycle )
        {
            return cycle;
        }
    }

    return 1000;
}



double
through_generate::estimate_virtual_dash_distance( int player_seen_pos_count,
                                                  int player_pos_count,
                                                  double player_real_max_speed )
{
    const int pos_count = std::min( 10, // Magic Number
                                    std::min( player_seen_pos_count,
                                              player_pos_count ) );
    const double max_speed = player_real_max_speed * 0.8; // Magic Number

    double d = 0.0;
    for ( int i = 1; i <= pos_count; ++i ) // start_value==1 to set the initial_value<1
    {
        //d += max_speed * std::exp( - (i*i) / 20.0 ); // Magic Number
        d += max_speed * std::exp( - (i*i) / 15.0 ); // Magic Number
    }

    return d;
}


