// Generated by gencpp from file comp313p_mapper/ChangeMapperState.msg
// DO NOT EDIT!


#ifndef COMP313P_MAPPER_MESSAGE_CHANGEMAPPERSTATE_H
#define COMP313P_MAPPER_MESSAGE_CHANGEMAPPERSTATE_H

#include <ros/service_traits.h>


#include <comp313p_mapper/ChangeMapperStateRequest.h>
#include <comp313p_mapper/ChangeMapperStateResponse.h>


namespace comp313p_mapper
{

struct ChangeMapperState
{

typedef ChangeMapperStateRequest Request;
typedef ChangeMapperStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ChangeMapperState
} // namespace comp313p_mapper


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::comp313p_mapper::ChangeMapperState > {
  static const char* value()
  {
    return "67481c27efef5721c8c4770c4dd3d307";
  }

  static const char* value(const ::comp313p_mapper::ChangeMapperState&) { return value(); }
};

template<>
struct DataType< ::comp313p_mapper::ChangeMapperState > {
  static const char* value()
  {
    return "comp313p_mapper/ChangeMapperState";
  }

  static const char* value(const ::comp313p_mapper::ChangeMapperState&) { return value(); }
};


// service_traits::MD5Sum< ::comp313p_mapper::ChangeMapperStateRequest> should match 
// service_traits::MD5Sum< ::comp313p_mapper::ChangeMapperState > 
template<>
struct MD5Sum< ::comp313p_mapper::ChangeMapperStateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::comp313p_mapper::ChangeMapperState >::value();
  }
  static const char* value(const ::comp313p_mapper::ChangeMapperStateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::comp313p_mapper::ChangeMapperStateRequest> should match 
// service_traits::DataType< ::comp313p_mapper::ChangeMapperState > 
template<>
struct DataType< ::comp313p_mapper::ChangeMapperStateRequest>
{
  static const char* value()
  {
    return DataType< ::comp313p_mapper::ChangeMapperState >::value();
  }
  static const char* value(const ::comp313p_mapper::ChangeMapperStateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::comp313p_mapper::ChangeMapperStateResponse> should match 
// service_traits::MD5Sum< ::comp313p_mapper::ChangeMapperState > 
template<>
struct MD5Sum< ::comp313p_mapper::ChangeMapperStateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::comp313p_mapper::ChangeMapperState >::value();
  }
  static const char* value(const ::comp313p_mapper::ChangeMapperStateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::comp313p_mapper::ChangeMapperStateResponse> should match 
// service_traits::DataType< ::comp313p_mapper::ChangeMapperState > 
template<>
struct DataType< ::comp313p_mapper::ChangeMapperStateResponse>
{
  static const char* value()
  {
    return DataType< ::comp313p_mapper::ChangeMapperState >::value();
  }
  static const char* value(const ::comp313p_mapper::ChangeMapperStateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // COMP313P_MAPPER_MESSAGE_CHANGEMAPPERSTATE_H
