// This code was taken from costmap_2d_ros from navigation stack.
// Original license:
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "luh_laser_watchdog/footprint_reader.h"
#include <XmlRpc.h>

using namespace luh_laser_watchdog;

Footprint FootprintReader::readFootprint(ros::NodeHandle &nh)
{
    std::string full_param_name;

    if( nh.searchParam( "laser_watchdog/concave_footprint", full_param_name ))
    {
        XmlRpc::XmlRpcValue footprint_xmlrpc;
        nh.getParam( full_param_name, footprint_xmlrpc );
        if( footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString )
        {
            if( readFootprintFromString( std::string( footprint_xmlrpc )))
            {
                //                writeFootprintToParam( nh );
            }
        }
        else if( footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray )
        {
            readFootprintFromXMLRPC( footprint_xmlrpc, full_param_name );
            //            writeFootprintToParam( nh );
        }
    }

    Footprint fp;
    for(uint i=0; i<footprint_.size(); i++)
    {
        Point2D p;
        p.x = footprint_[i].x;
        p.y = footprint_[i].y;

        fp.push_back(p);
    }

    return fp;
}

bool FootprintReader::readFootprintFromString( const std::string& footprint_string )
{
    std::string error;
    std::vector<std::vector<float> > vvf = parseVVF( footprint_string, error );
    if( error != "" )
    {
        ROS_ERROR( "Error parsing footprint parameter: '%s'", error.c_str() );
        ROS_ERROR( "  Footprint string was '%s'.", footprint_string.c_str() );
        return false;
    }

    // convert vvf into points.
    if( vvf.size() < 3 )
    {
        ROS_ERROR( "You must specify at least three points for the robot footprint, reverting to previous footprint." );
        return false;
    }
    std::vector<geometry_msgs::Point> points;
    points.reserve( vvf.size() );
    for( unsigned int i = 0; i < vvf.size(); i++ )
    {
        if( vvf[ i ].size() == 2 )
        {
            geometry_msgs::Point point;
            point.x = vvf[ i ][ 0 ];
            point.y = vvf[ i ][ 1 ];
            point.z = 0;
            points.push_back( point );
        }
        else
        {
            ROS_ERROR( "Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                       int( vvf[ i ].size() ));
            return false;
        }
    }

    //    setUnpaddedRobotFootprint( points );
    footprint_ = points;
    return true;
}


void FootprintReader::readFootprintFromXMLRPC( XmlRpc::XmlRpcValue& footprint_xmlrpc,
                              const std::string& full_param_name )
{
    // Make sure we have an array of at least 3 elements.
    if( footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            footprint_xmlrpc.size() < 3 )
    {
        ROS_FATAL( "The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                   full_param_name.c_str(), std::string( footprint_xmlrpc ).c_str() );
        throw std::runtime_error( "The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    std::vector<geometry_msgs::Point> footprint;
    geometry_msgs::Point pt;

    for( int i = 0; i < footprint_xmlrpc.size(); ++i )
    {
        // Make sure each element of the list is an array of size 2. (x and y coordinates)
        XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
        if( point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != 2 )
        {
            ROS_FATAL( "The footprint (parameter %s) must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                       full_param_name.c_str() );
            throw std::runtime_error( "The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form" );
        }

        pt.x = getNumberFromXMLRPC( point[ 0 ], full_param_name );
        pt.y = getNumberFromXMLRPC( point[ 1 ], full_param_name );

        footprint.push_back( pt );
    }

//    setUnpaddedRobotFootprint( footprint );
    footprint_ = footprint;
}

/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<float> > FootprintReader::parseVVF( const std::string& input, std::string& error_return )
{
    std::vector<std::vector<float> > result;

    std::stringstream input_ss( input );
    int depth = 0;
    std::vector<float> current_vector;
    while( !!input_ss && !input_ss.eof() )
    {
        switch( input_ss.peek() )
        {
        case EOF:
            break;
        case '[':
            depth++;
            if( depth > 2 )
            {
                error_return = "Array depth greater than 2";
                return result;
            }
            input_ss.get();
            current_vector.clear();
            break;
        case ']':
            depth--;
            if( depth < 0 )
            {
                error_return = "More close ] than open [";
                return result;
            }
            input_ss.get();
            if( depth == 1 )
            {
                result.push_back( current_vector );
            }
            break;
        case ',':
        case ' ':
        case '\t':
            input_ss.get();
            break;
        default: // All other characters should be part of the numbers.
            if( depth != 2 )
            {
                std::stringstream err_ss;
                err_ss << "Numbers at depth other than 2. Char was '" << char( input_ss.peek() ) << "'.";
                error_return = err_ss.str();
                return result;
            }
            float value;
            input_ss >> value;
            if( !!input_ss )
            {
                current_vector.push_back( value );
            }
            break;
        }
    }

    if( depth != 0 )
    {
        error_return = "Unterminated vector string.";
    }
    else
    {
        error_return = "";
    }

    return result;
}

double FootprintReader::getNumberFromXMLRPC( XmlRpc::XmlRpcValue& value, const std::string& full_param_name )
{
  // Make sure that the value we're looking at is either a double or an int.
  if( value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble )
  {
    std::string& value_string = value;
    ROS_FATAL( "Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str() );
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}
