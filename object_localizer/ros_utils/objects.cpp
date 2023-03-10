#include "ros_utils/object_map/objects.h"

using namespace object_localizer;

Objects::Objects() {}

void Objects::add_object(Object object) 
{ 
	this->emplace_back(object); 
}

void Objects::print_elements()
{
	for(auto it = this->begin(); it != this->end(); it++){
		it->print_element();
	}
    std::cout << std::endl;
}

Object Objects::get_nearest_object(double x,double y,double& sim)
{
	std::vector<double> sim_list(this->size(),0.0);
	for(size_t i = 0; i < this->size(); i++) sim_list[i] = this->at(i).get_similarity(x,y);
	int max_index = std::distance(sim_list.begin(),std::max_element(sim_list.begin(),sim_list.end()));
	sim = sim_list[max_index];
	this->at(max_index).is_observed = true;
	return this->at(max_index);
}