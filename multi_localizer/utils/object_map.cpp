#include "utils/object_map/object_map.h"

using namespace multi_localizer;

ObjectMap::ObjectMap() {}

void ObjectMap::add_init_object(std::string name,Object object)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name) it->second->add_init_object(object);
    }
}

void ObjectMap::clear_objects_data()
{
    for(auto it = this->begin(); it != this->end(); it++) it->second->clear();
}

void ObjectMap::all_objects_are_not_observed()
{
    for(auto it = this->begin(); it != this->end(); it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            sit->has_observed = false;
        }
    }
}

Object ObjectMap::get_highly_similar_object(std::string name,double x,double y,double& sim)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name){
            return it->second->get_highly_similar_object(x,y,sim);
        }
    }
    ROS_ERROR("No corresponding object");
    sim = -1e10;
    return Object();
}

// for debug
void ObjectMap::print_object_params()
{
    for(auto it = this->begin(); it != this->end(); it++) it->first->print_element();
}

// for debug
void ObjectMap::print_elements()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->first->print_param();
        it->second->print_elements();
    }
}