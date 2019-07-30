#include "Common_include.h"
#include "Frame.h"
#include "MapPoint.h"

namespace Mono_vo
{

MapPoint::MapPoint()
: id_(-1), pos_(Point3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
{
}


MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
    );
}


unsigned long MapPoint::factory_id_ = 0;

}