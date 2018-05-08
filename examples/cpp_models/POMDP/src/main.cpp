/*
 * main.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: bingyu
 */
#include <despot/simple_tui.h>
#include "POMDP.h"

using namespace despot;

class TUI : public SimpleTUI{
public:
	TUI(){
	}

	DSPOMDP* InitializeModel(option::Option* options){
		DSPOMDP* model  = new POMDP_Plan();
		return model;
	}

	void InitializeDefaultParameters(){
		Globals::config.num_scenarios = 20;
		Globals::config.time_per_move = 1.0/3;
		Globals::config.sim_len = 90;
		Globals::config.search_depth = 45;
	}
};

int main(int argc, char* argv[]){
	return TUI().run(argc, argv);

}
