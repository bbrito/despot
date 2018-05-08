//============================================================================
// Name        : POMDP.cpp
// Author      : bingyu
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include "POMDP.h"
#include <cmath>
#include <map>
#include "eigen3/Eigen/Dense"
#include "Clothoid.h"
#include "spline.h"

using namespace std;
using namespace Eigen;

const double LENGTH=4;
const double WIDTH=2;
const double PI=3.1415;
const double SPEED_DESIRED = 8;
const double V_max = 10;
const double REWARD_ACC = -1;
const double REWARD_COLL = -1000;

namespace despot{
/*==================================
 * POMDP_Plan state class
 ====================================*/
POMDP_Plan_State::POMDP_Plan_State(){
}
POMDP_Plan_State::POMDP_Plan_State(std::vector<double> _state_R, std::vector<double> _state_A){
	state_R = _state_R;
	state_A = _state_A;
}
POMDP_Plan_State::~POMDP_Plan_State(){
}

string POMDP_Plan_State::text()const{
	return "ego-veh position = " + to_string(state_R) + "obstacle position = " +
			to_string(state_A);
}

class POMDPSmartParticleUpperBound : public ParticleUpperBound, POMDP_Plan{
protected:
	const DSPOMDP* model_;
public:
	POMDPSmartParticleUpperBound(const DSPOMDP* model) : model_(model){}
	double Value(const State& state) const{
		const POMDP_Plan_State& pomdp_state = static_cast<const POMDP_Plan_State&>(state);
		double reward_goal = Gausspdf(100-pomdp_state.state_A[5]);
		return reward_goal * Globals::Discount((100-pomdp_state.state_A[5])/V_max);
	}
};

class POMDPSmartParticleLowerBound : public ParticleLowerBound{
protected:
	const DSPOMDP* model_;
public:
	POMDPSmartParticleLowerBound(const DSPOMDP* model):
		ParticleLowerBound(model), model_(model){}
	ValuedAction Value(const vector<State*>& particles) const{
		const POMDP_Plan_State& pomdp_state = static_cast<const POMDP_Plan_State&>(*particles[0]);
		int action = Random::RANDOM.NextInt(0,2);

		return ValuedAction(action, State::Weight(particles)*(REWARD_COLL * pomdp_state.state_A[3] + REWARD_ACC / (1-Globals::Discount())));
	}
};
/*================================
 * POMDP Smart Policy
 ================================*/
class POMDPSmartPolicy : public Policy {
public:
	POMDPSmartPolicy(const DSPOMDP* model, ParticleLowerBound* bound) : Policy(model,bound){}
	int Action(const vector<State*>& particles, RandomStreams& streams, History& history) const{
		const POMDP_Plan_State& state = static_cast<POMDP_Plan_State&>(*particles[0]);
		int smart_action;

		if (history.Size()>0){
			int action = history.LastAction();
			OBS_TYPE observation = history.LastObservation();
			double x_R = state.state_R[0];
			double y_R = state.state_R[1];
			double v_R = state.state_R[3];
			double x_A = state.state_A[0];
			double y_A = state.state_A[1];
			double v_A = state.state_A[3];

			double dist_RA = pow((x_R-x_A),2) +pow((y_R-y_A),2);


			if (1.5*LENGTH < dist_RA && dist_RA < 2.5*LENGTH){
				if (v_A > 2){
					return smart_action = 1;
				}
				else
					return smart_action = 2;
			}
			else{
				if (dist_RA <= 1.5*LENGTH){
					if (v_A != 0 )
						return smart_action = 1;
					else
						return smart_action = 2;
				}
				else
					return smart_action = 0;
			}
		}

		return smart_action = 0;


	}
};

/*====================================
 * POMDP_Plan class
 =====================================*/

POMDP_Plan::POMDP_Plan(){
	vector<double> x_A_0 = {40, 0};
	vector<double> y_A_0 = {8, 8};
	vector<double> theta_A_0 = {PI, PI};
	vector<double> x_A_1 = {40, 30, 23, 23};
	vector<double> y_A_1 = {8, 8, 0, -50};
	vector<double> theta_A_1 = {PI, PI, -PI/2, -PI/2};
	vector<double> x_R = {27, 27, 17, -300};
	vector<double> y_R = {-12, -2, 8, 8};
	vector<double> theta_R = {PI/2, PI/2, PI, PI};

	//cout << x_A_0[0] << endl;

	REF_PATH_A_0 = Ref_path(x_A_0, y_A_0, theta_A_0);
	//cout << REF_PATH_A_0.size() << endl;
	REF_PATH_A_1 = Ref_path(x_A_1, y_A_1, theta_A_1);

	REF_PATH_R = Ref_path(x_R, y_R, theta_R);

	/*for (int i = 0; i< 50; i++){
		cout << REF_PATH_A_0[0](i) << " " << REF_PATH_A_0[1](i) << endl;
	}*/

	Init();

}

// Actions
int POMDP_Plan::NumActions() const{
	return 3;
}

//Reference path
vector<tk::spline> POMDP_Plan::Ref_path(vector<double> x, vector<double> y, vector<double> theta) const{

	double k, dk, L;
	vector<double> X(10), Y(10);
	vector<double> X_all, Y_all, S_all;
	double total_length=0;
	int n_clothoid = 20;
	S_all.push_back(0);



	for (int i = 0; i < x.size()-1; i++){
		Clothoid::buildClothoid(x[i], y[i], theta[i], x[i+1], y[i+1], theta[i+1], k, dk, L);
		//cout << k << " " << dk << " " << L << endl;
		Clothoid::pointsOnClothoid(x[i], y[i], theta[i], k, dk, L, n_clothoid, X, Y);
		if (i==0){
			X_all.insert(X_all.end(), X.begin(), X.end());
			Y_all.insert(Y_all.end(), Y.begin(), Y.end());
		}
		else{
			X.erase(X.begin()+0);
			Y.erase(Y.begin()+0);
			X_all.insert(X_all.end(), X.begin(), X.end());
			Y_all.insert(Y_all.end(), Y.begin(), Y.end());
		}
		total_length = total_length + L;
		for (int j=1; j< n_clothoid; j++){
				S_all.push_back(S_all[j-1+i*(n_clothoid-1)]+L/(n_clothoid-1));
		}

	}

	tk::spline ref_path_x, ref_path_y;
	ref_path_x.set_points(S_all, X_all);
	ref_path_y.set_points(S_all, Y_all);
	vector<tk::spline> ref_path(2);
	ref_path[0] = ref_path_x;
	ref_path[1] = ref_path_y;

	double dist = total_length / 80;

	vector<double> ss(80),xx(80),yy(80);

	for (int i=0; i<80; i++){
		ss[i] = dist *i;
		xx[i] = ref_path_x(ss[i]);
		yy[i] = ref_path_y(ss[i]);
	}
	ref_path_x.set_points(ss,xx);
	ref_path_y.set_points(ss,yy);

	//cout << "ref_path" << ref_path.size() << endl;

	return ref_path;
}

// Find the reference steering angle
double POMDP_Plan::FindSteer_A(vector<double>& state_A)const{

	double s = FindSegmentation_A(state_A);

	double dx, dy, orient;

	if (state_A[4] == 0){
		dx = REF_PATH_A_0[0].deriv(1, s);
		dy = REF_PATH_A_0[1].deriv(1, s);
		orient = atan2(dy,dx);
	}
	else{
		dx = REF_PATH_A_1[0].deriv(1, s);
		dy = REF_PATH_A_1[1].deriv(1, s);
		orient = atan2(dy,dx);
	}

	//cout << "orintation_A: " << orient << endl;
	return orient;
}

double POMDP_Plan::FindSteer_R(vector<double>& state_R)const{

	double s = FindSegmentation_R(state_R);

	double dx = REF_PATH_R[0].deriv(1, s);
	double dy = REF_PATH_R[1].deriv(1, s);
	double orient = atan2(dy, dx);

	//cout << "orintation_R: " << orient << endl;
	return orient;
}

double POMDP_Plan::FindSegmentation_A(vector<double>& state_A)const{


	double s_guess = state_A[5];
	double goal = state_A[4];
	double x = state_A[0];
	double y = state_A[1];
	int window = 5;
	int n_sample = 100;
	double lower, upper;
	double x_ref, y_ref;
	double norm_i, norm_min=25;
	int index_min;
	vector<tk::spline> ref_path;

	if (s_guess-window < 0)
		lower = 0;
	else
		lower = s_guess-window;
	if (s_guess+window < 1000)
		upper = s_guess+window;
	else
		upper = 1000;
	vector<double> s_i = linspace(lower, upper, n_sample);

	if (goal == 0)
		ref_path = REF_PATH_A_0;
	else
		ref_path = REF_PATH_A_1;

	for (int i = 0; i< n_sample; i++){
		x_ref = ref_path[0](s_i[i]);
		y_ref = ref_path[1](s_i[i]);
		norm_i = sqrt(pow(x-x_ref,2) + pow(y-y_ref,2));
		if (norm_i < norm_min){
			norm_min = norm_i;
			index_min = i;
		}
	}

	state_A[5] = s_i[index_min];

	//cout << "find seg_A:" << s_i[index_min] << endl;
	return s_i[index_min];
}

double POMDP_Plan::FindSegmentation_R(vector<double>& state_R)const{
	double s_guess = state_R[4];
	double x = state_R[0];
	double y = state_R[1];
	int window = 5;
	int n_sample = 100;
	double lower, upper;
	double x_ref, y_ref;
	double norm_i, norm_min=20;
	int index_min;
	vector<tk::spline> ref_path = REF_PATH_R;

	if (s_guess-window < 0)
		lower = 0;
	else
		lower = s_guess-window;
	if (s_guess+window < 1000)
		upper = s_guess+window;
	else
		upper = 1000;
	vector<double> s_i = linspace(lower, upper, n_sample);

	for (int i = 0; i< n_sample; i++){
		x_ref = ref_path[0](s_i[i]);
		y_ref = ref_path[1](s_i[i]);
		norm_i = sqrt(pow(x-x_ref,2) + pow(y-y_ref,2));
		if (norm_i < norm_min){
			norm_min = norm_i;
			index_min = i;
		}
	}

	state_R[4] = s_i[index_min];
	//cout << "find seg R " << s_i[index_min] << endl;
	return s_i[index_min];
}

inline Matrix2d POMDP_Plan::Rotation(double theta)const{
	Matrix2d rot;
	rot(0,0) = cos(theta);
	rot(0,1) = -sin(theta);
	rot(1,0) = sin(theta);
	rot(1,1) = rot(0,0);
	return rot;
}
inline double POMDP_Plan::Gausspdf(double d)const{
	double pdf;
	double beta=100, sigma=4;
	return pdf = beta*exp(-d/2/pow(sigma,2));
}

// Dynamics
vector<double> POMDP_Plan::Dynamics_A(vector<double> state_A, int action) const{
	vector<double> next_state_A(6);
	double x, y, theta, v, goal;
	double t=0.05;// time interval

	x = state_A[0];
	y = state_A[1];
	theta = state_A[2];
	v = state_A[3];
	goal = state_A[4];

	switch (action){
	case Acc : next_state_A[3] = (v + 1*t) < V_max ? v+1*t : 5;break;
	case Dec : next_state_A[3] = (v - 1*t) > 0 ? v-1*t : 0;break;
	case Cur : next_state_A[3] = v;break;
	}

	//cout << "vel A: " << next_state_A[3] << endl;
	//cout << "goal: " << goal << endl;

	if (goal == 2.0){
		next_state_A = state_A;
	}
	else{
		next_state_A[0] = x + v*t*cos(theta);
		next_state_A[1] = y + v*t*sin(theta);
		next_state_A[2] = FindSteer_A(state_A);
		next_state_A[4] = goal;
		next_state_A[5] = v*t + state_A[5];
	}

	//cout << next_state_A[4] << endl;
	return next_state_A;

}

vector<double> POMDP_Plan::Dynamics_R(vector<double> state_R) const{
	double x,y,theta,v;
	double t=0.05;
	vector<double> next_state_R(5);

	x = state_R[0];
	y = state_R[1];
	theta = state_R[2];
	v = state_R[3];



	next_state_R[0] = x + v*t*cos(theta);

	next_state_R[1] = y + v*t*sin(theta);

	next_state_R[2] = FindSteer_R(state_R);

	next_state_R[3] = v; // constant vel in each planning cycle
	next_state_R[4] = v*t + state_R[4];


	return next_state_R;

}
void POMDP_Plan::Init(){
	int x_R, y_R;
	int x_A, y_A;
	int x_R_bound_l = -10;
	int x_R_bound_u = 30;
	int y_R_bound_l = -13;
	int y_R_bound_u = 10;
	int x_A_bound_l = -10;
	int x_A_bound_u = 40;
	int y_A_bound_l = -30;
	int y_A_bound_u = 10;
	int count = 1;
	vector<int> s(4);

	for(x_R = x_R_bound_l; x_R <= x_R_bound_u; x_R = x_R + 1){
		for(y_R = y_R_bound_l; y_R <= y_R_bound_u; y_R = y_R + 1){
			for(x_A = x_A_bound_l; x_A <= x_A_bound_u; x_A=x_A+1){
				for(y_A = y_A_bound_l; y_A <= y_A_bound_u; y_A = y_A+1){
					s[0] = x_R; s[1] = y_R; s[2] = x_A; s[3] = y_A;
					obs_[s] = count;
					++count;
				}
			}
		}
	}


}
int POMDP_Plan::MakeObservation(const POMDP_Plan_State _pomdp_state) const{
	int observation;
	vector<double> _state_R = _pomdp_state.state_R;
	vector<double> _state_A = _pomdp_state.state_A;
	//double s_R, s_A;
	vector<int> s;
	//int p, m;
	//double n;

//	//Resolution of position 1m, resolution of velocity 0.1m/d
//	s_R = _state_R[4];
//	s_A = _state_A[5];
//	m = int(floor(s_R*100)) / 5;
//	p = int(floor(s_R*100)) % 5;
//	if (p<3)
//		n = m*5.0;
//	else
//		n = (m+1)*5.0;
//
//	//cout << "appro state " << n << endl;
//
//	s.push_back(n);
//
//	m = int(floor(s_A*100)) / 5;
//	p = int(floor(s_A*100)) % 5;
//	if (p<3)
//		n = m*5.0;
//	else
//		n = (m+1)*5.0;
//
//	//cout << "appro state " << n << endl;
//
//	s.push_back(n);
//	observation = obs_.at(s);
	double x_R = floor(_state_R[0]);
	double y_R = floor(_state_R[1]);
	double x_A = floor(_state_A[0]);
	double y_A = floor(_state_A[1]);
	s.push_back((int)x_R);
	s.push_back((int)y_R);
	s.push_back((int)x_A);
	s.push_back((int)y_A);
	//cout << s[0] << " " << s[1] << " " << s[2] << " " << s[3] << endl;
	observation = obs_.at(s);

	return observation;
}
// Deterministic simulative model P(z',o | z, u)
bool POMDP_Plan::Step(State& state, double rand_num, int action, double& reward,
		OBS_TYPE& obs) const {
	// s, s' are represented by the same pomdp_state
	POMDP_Plan_State& pomdp_state = static_cast <POMDP_Plan_State&> (state);
	vector<double>& state_R = pomdp_state.state_R;
	vector<double>& state_A = pomdp_state.state_A;

	//cout << "action: " << action << endl;
	//cout << "goal: " << state_A[4] << endl;
	vector<double> next_state_R(4), next_state_A(5);
	next_state_R = Dynamics_R(state_R);
	next_state_A = Dynamics_A(state_A, action);

	//cout << "next state R: " << next_state_R[4] << endl;
	//cout << "next state A: " << next_state_A[5] << endl;

	state_R = next_state_R;
	state_A = next_state_A;

	obs = MakeObservation(pomdp_state);
	//cout << "obs: " << obs << endl;

	Matrix2d rot(2,2);
	rot(0,0) = cos(next_state_R[2]);
	rot(0,1) = -sin(next_state_R[2]);
	rot(1,0) = sin(next_state_R[2]);
	rot(1,1) = rot(0,0);

	Matrix<double, 2, 4> circle_pos;
	circle_pos(0,0) = -3/8*LENGTH;
	circle_pos(0,1) = -1/8*LENGTH;
	circle_pos(0,2) = 1/8*LENGTH;
	circle_pos(0,3) = 3/8*LENGTH;
	circle_pos(1,0) = 0;
	circle_pos(1,1) = 0;
	circle_pos(1,2) = 0;
	circle_pos(1,3) = 0;

	Matrix<double, 2, 4> position;
	position(0,0) = next_state_R[0];
	position(1,0) = next_state_R[1];
	position(0,1) = position(0,0);
	position(1,1) = position(1,0);
	position(0,2) = position(0,0);
	position(1,2) = position(1,0);
	position(0,3) = position(0,0);
	position(1,3) = position(1,0);

	Matrix<double, 2, 4> circle_R = position + Rotation(next_state_R[2]) * circle_pos;
	Matrix2d rotation_A = Rotation(next_state_A[2]);

	double x_semi = sqrt(2)/2*LENGTH + sqrt(pow(LENGTH,2) / 64 + pow(WIDTH,2) / 4) + 0.1;
	double y_semi = sqrt(2)/2*WIDTH + sqrt(pow(LENGTH,2) / 64 + pow(WIDTH,2) / 4) + 0.1;
	Vector2d vector(x_semi, y_semi);
	DiagonalMatrix<double, 2> semi;
	semi.diagonal() = vector;
	Vector2d pos_A(next_state_A[0], next_state_A[1]);
	Vector2d vector_dis;
	double flag_coll;
	double reward_coll = 0;
	for(int i=0; i<4; i++){
		vector_dis = circle_R.col(i) - pos_A;
		flag_coll = vector_dis.transpose() * rotation_A * semi * rotation_A.transpose() * vector_dis;
		if (flag_coll <= 1){
			reward_coll = REWARD_COLL*next_state_A[3];
			break;
		}
		//cout << "collision: " << flag_coll << endl;
	}
	double reward_goal = Gausspdf(100 - next_state_A[5]);
	//cout << "reward goal: " << reward_goal << endl;
	double reward_acc = (action == Acc || action == Dec) ? REWARD_ACC : 0;
	//double reward_speed = abs(next_state_A[3]-SPEED_DESIRED)/SPEED_DESIRED;

	reward = reward_coll + reward_goal + reward_acc;
	//cout << "reward: " << reward << endl;
	if (flag_coll <= 1 || next_state_A[5] >= 100) return true;
	else return false;
}
// Observation probability O(z | s', a), incorporating the motion uncertainty of obstacle vehicle
double POMDP_Plan::ObsProb(OBS_TYPE obs, const State& state, int action) const{
	const POMDP_Plan_State& pomdp_state = static_cast<const POMDP_Plan_State&>(state);
	if (pomdp_state.state_A[4] == 2 && action != Dec){
		return (obs==MakeObservation(pomdp_state)) ? 0.5 : 0.5;
	}
	if (pomdp_state.state_A[4] == 2 && action == Dec)
		return (obs == MakeObservation(pomdp_state)) ? 0.4 : 0.6;

	if (pomdp_state.state_A[0] > 27){
		return (obs == MakeObservation(pomdp_state)) ? 0.8 : 0.2;
	}

	//cout << "Belief: " << pomdp_state.state_A[4];

	return obs == MakeObservation(pomdp_state);
}

State* POMDP_Plan::CreateStartState(std::string type) const{
	POMDP_Plan_State* start_state = memory_pool_.Allocate();
	vector<double> state_R(5), state_A(6);
	state_R = {27, -10, PI/2, 5, 0};
	state_A = {40, 8 ,-PI, 5, 0, 0};
	double prob = Random::RANDOM.NextDouble();
	if (prob < 0)
		state_A[4] = 0;
	else
		state_A[4] = 1;


	start_state->state_A = state_A;
	start_state->state_R = state_R;

	return start_state;
}

Belief* POMDP_Plan::InitialBelief(const State* start, std::string type) const{

	//int N = 3000;
	vector<State*> particles;

	/*for(int i = 0; i < N; i++){
		particles[i] = CreateStartState();
		particles[i]->weight = 1.0 / N;
	}*/
	POMDP_Plan_State* straight_state = static_cast<POMDP_Plan_State*>(Allocate(-1,0.45));
	straight_state->state_R = {27, -10, PI/2, 5, 0};
	straight_state->state_A = {40, 8 ,-PI, 5, 0, 0};
	particles.push_back(straight_state);

	POMDP_Plan_State* turning_state = static_cast<POMDP_Plan_State*>(Allocate(-1,0.45));
	turning_state->state_R = {27, -10, PI/2, 5, 0};
	turning_state->state_A = {40, 8 ,-PI, 5, 1, 0};
	particles.push_back(turning_state);

	POMDP_Plan_State* stop_state = static_cast<POMDP_Plan_State*>(Allocate(-1,0.1));
	stop_state->state_R = {27, -10, PI/2, 5, 0};
	stop_state->state_A = {40, 8 ,-PI, 5, 2, 0};
	particles.push_back(stop_state);
	return new ParticleBelief(particles, this);
}

double POMDP_Plan::GetMaxReward() const{
	return 500;
}


ScenarioUpperBound* POMDP_Plan::CreateScenarioUpperBound(string name, string particle_bound_name) const{
	return new POMDPSmartParticleUpperBound(this);
}

ValuedAction POMDP_Plan::GetMinRewardAction() const{
	return ValuedAction(Dec, 0);
}

ParticleLowerBound* POMDP_Plan::CreateParticleLowerBound(string name)const{
	return new POMDPSmartParticleLowerBound(this);
}

ScenarioLowerBound* POMDP_Plan::CreateScenarioLowerBound(string name,string particle_bound_name) const{
	return new POMDPSmartPolicy(this, CreateParticleLowerBound(particle_bound_name));
}



State* POMDP_Plan::Allocate(int state_id, double weight) const{
	POMDP_Plan_State* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}
State* POMDP_Plan::Copy(const State* particle) const{
	POMDP_Plan_State* state = memory_pool_.Allocate();
	*state = *static_cast<const POMDP_Plan_State*> (particle);
	state->SetAllocated();
	return state;
}
void POMDP_Plan::Free(State* particle) const{
	memory_pool_.Free(static_cast<POMDP_Plan_State*>(particle));
}
int POMDP_Plan::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

void POMDP_Plan::PrintState(const State& state, std::ostream& out) const{
	const POMDP_Plan_State& pomdp_state = static_cast<const POMDP_Plan_State&> (state);
	out << "state_R: " << "x: " << pomdp_state.state_R[0] << " y: " << pomdp_state.state_R[1] <<" theta: " <<
			pomdp_state.state_R[2] << " velocity: " << pomdp_state.state_R[3] << " progress: " << pomdp_state.state_R[4] << endl;
	out << "state_A: " << "x: " << pomdp_state.state_A[0] << " y: " << pomdp_state.state_A[1] <<" theta: " <<
				pomdp_state.state_A[2] << " velocity: " << pomdp_state.state_A[3] << " goal: " << pomdp_state.state_A[4] << " progress: " << pomdp_state.state_A[5] << endl;

}
void POMDP_Plan::PrintBelief(const Belief& belief, vector<double>& goal_probs, std::ostream& out) const{
	const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
	goal_probs.resize(3);
	goal_probs[0] = 0; goal_probs[1] = 0; goal_probs[2] = 0;
	for (int i=0; i < particles.size(); i++){
		State* particle = particles[i];
		const POMDP_Plan_State* state = static_cast<const POMDP_Plan_State*>(particle);
		if (state->state_A[4] == 0)
			goal_probs[0] += particle->weight;
		else{
			if (state->state_A[4] == 1)
				goal_probs[1] += particle->weight;
			else
				goal_probs[2] += particle->weight;
		}
	}
	out << "Goal belief: " << "Straight: " << goal_probs[0] << "Turning: " << goal_probs[1] <<
			"Stopping: " << goal_probs[2] <<endl;
}
void POMDP_Plan::PrintObs(const State& state, OBS_TYPE observation,std::ostream& out) const{

}
void POMDP_Plan::PrintAction(int action, std::ostream& out) const{
 out << action << endl;

}
}
