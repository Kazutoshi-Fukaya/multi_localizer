#include "utils/object_map/objects.h"

using namespace multi_localizer;

Objects::Objects() {}

void Objects::add_init_object(Object object) { this->emplace_back(object); }

Object Objects::get_highly_similar_object(double x,double y,double& sim)
{
	std::vector<double> sim_list(this->size(),0.0);
	for(size_t i = 0; i < this->size(); i++){
		sim_list[i] = this->at(i).get_similarity(x,y);
    }
	int max_index = std::distance(sim_list.begin(),std::max_element(sim_list.begin(),sim_list.end()));
    sim = sim_list[max_index];
	this->at(max_index).has_observed = true;
	return this->at(max_index);
}

// for debug
void Objects::print_elements()
{
	for(auto it = this->begin(); it != this->end(); it++) it->print_element();
    std::cout << std::endl;
}