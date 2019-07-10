#include<iostream>
#include<cmath>
#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/angle_deg.h>
#include <rcsc/geom/rect_2d.h>
#include <rcsc/geom/size_2d.h>

using namespace rcsc;

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
                        int kicker_unum,
                        int self_unum,
                        bool kickable,
                        bool play_on,
                        bool penalty_kick_mode );

double calc_first_term_geom_series( const double & sum,
                                    const double & r,
                                    const int len );

Vector2D inertia_n_step_point( Vector2D first_ball_pos,
                               Vector2D first_ball_vel,
                               int cycle,
                               double ball_decay );

Vector2D player_interia_pos( Vector2D player_pos,
                             Vector2D player_vel,
                             int cycle );


int main()
{
    rcsc::Vector2D a(-10,10);
    std::cout<< a.absX() <<"\n";
}


bool
createPassCommon( const rcsc::Vector2D & receive_point,
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

    CooperativeAction::Ptr pass( new Pass( M_passer->unum(),
                                           receiver.player_->unum(),
                                           receive_point,
                                           first_ball_speed,
                                           step + kick_count,
                                           kick_count,
                                           FieldAnalyzer::to_be_final_action( wm ),
                                           description ) );


    return true;
}

double
calc_first_term_geom_series( const double & sum,
                             const double & r,
                             const int len )
{
    return sum * ( 1.0 - r ) / ( 1.0 - std::pow( r, len ) );
}


int
predict_kick_count( const double & first_ball_speed,
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
        Vector2D max_vel = KickTable::calc_max_velocity( ball_move_angle,
                                                         wm.self().kickRate(),
                                                         wm.ball().vel() );
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
calc_max_velocity( const double & krate,
                   const Vector2D & ball_vel,
                   const AngleDeg target_angle )
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


int
StrictCheckPassGenerator::predictOpponentsReachStep( const WorldModel & wm,
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

int
predictOpponentReachStep( const Vector2D & first_ball_pos,
                          const Vector2D & first_ball_vel,
                          const Vector2D & receive_point,
                          const Vector2D & opponent_pos,
                          const Vector2D & opponent_vel,
                          const AngleDeg & ball_move_angle,
                          const double opp_kick_ablearea,
                          const double offside_line_x,
                          bool goalie,
                          const int max_cycle,
                          const char M_pass_type )
{
    static const Rect2D penalty_area( Vector2D( penalty_area_length,
                                                -penalty_area_width / 2 ),
                                      Size2D( penalty_area_length,
                                              penalty_area_width ) );
    static const double CONTROL_AREA_BUF = 0.15;

    const int min_cycle = estimate_min_reach_cycle( opponent_pos,
                                                    first_ball_pos,
                                                    ball_move_angle );
    if ( min_cycle < 0 )
    {
        return 1000;
    }

    for ( int cycle = std::max( 1, min_cycle ); cycle <= max_cycle; ++cycle )
    {
        const Vector2D ball_pos = inertia_n_step_point( first_ball_pos,
                                                        first_ball_vel,
                                                        cycle,
                                                        ball_decay );
        const double control_area = ( goalie
                                      && penalty_area.contains( ball_pos )
                                      ? catch_ablearea
                                      : opp_kick_ablearea );

        const Vector2D inertia_pos = player_interia_pos( opponent_pos, opponent_vel, cycle );
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
            dash_dist -= opponent.bonus_distance_;
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
             * ( cycle + std::min( opponent.player_->posCount(), 5 ) ) )
        {
            continue;
        }

        //
        // dash
        //

        int n_dash = ptype->cyclesToReachDistance( dash_dist );

        if ( n_dash > cycle + opponent.player_->posCount() )
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

        /*
        int n_step = ( n_turn == 0
                       ? n_turn + n_dash
                       : n_turn + n_dash + 1 ); // 1 step penalty for observation delay
                       */
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

int
estimate_min_reach_cycle( const rcsc::Vector2D & player_pos,
                          const rcsc::Vector2D & target_first_point,
                          const rcsc::AngleDeg & target_move_angle )
{
    rcsc::Vector2D target_to_player = ( player_pos - target_first_point ).rotatedVector( -target_move_angle );
    return ( target_to_player.x < -1.0
             ? -1
             : std::max( 1, static_cast< int >( std::floor( target_to_player.absY() / player_speed_max ) ) ) );
}


Vector2D inertia_n_step_point( Vector2D first_ball_pos,
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

Vector2D player_interia_pos( Vector2D player_pos,
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

/*
double
estimate_virtual_dash_distance( const rcsc::AbstractPlayerObject * player )
{
    const int pos_count = std::min( 10, // Magic Number
                                    std::min( player->seenPosCount(),
                                              player->posCount() ) );
    const double max_speed = player->playerTypePtr()->realSpeedMax() * 0.8; // Magic Number

    double d = 0.0;
    for ( int i = 1; i <= pos_count; ++i ) // start_value==1 to set the initial_value<1
    {
        //d += max_speed * std::exp( - (i*i) / 20.0 ); // Magic Number
        d += max_speed * std::exp( - (i*i) / 15.0 ); // Magic Number
    }

    return d;
}
*?

