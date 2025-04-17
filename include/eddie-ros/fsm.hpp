#ifndef EDDIE_ROS_FSM_HPP
#define EDDIE_ROS_FSM_HPP

#include "coord2b/functions/fsm.h"
#include "coord2b/functions/event_loop.h"

// sm enums
enum e_events {
	E_STEP = 0,
	E_CONFIGURE_ENTERED,
	E_CONFIGURE_EXIT,
	E_IDLE_ENTERED,
	E_IDLE_EXIT_COMPILE,
	E_IDLE_EXIT_EXECUTE,
	E_COMPILE_ENTERED,
	E_COMPILE_EXIT,
	E_EXECUTE_ENTERED,
	NUM_EVENTS
};

enum e_states { S_START = 0, S_CONFIGURE, S_IDLE, S_COMPILE, S_EXECUTE, S_EXIT, NUM_STATES };

enum e_transitions {
	T_START_CONFIGURE = 0,
	T_CONFIGURE_IDLE,
	T_IDLE_IDLE,
	T_IDLE_COMPILE,
	T_IDLE_EXECUTE,
	T_COMPILE_EXECUTE,
	T_EXECUTE_EXECUTE,
	NUM_TRANSITIONS
};

enum e_reactions {
	R_CONFIGURE_IDLE = 0,
	R_IDLE_EXECUTE,
	R_IDLE_COMPILE,
	R_COMPILE_EXECUTE,
	R_ALWAYS_TRUE,
	NUM_REACTIONS
};

// sm structs
inline struct state states[NUM_STATES] = {
	{.name = "Start"},
	{.name = "Configure"},
	{.name = "Idle"},
	{.name = "Compile"},
	{.name = "Execute"},
	{.name = "Exit"}
};

inline struct transition transitions[NUM_TRANSITIONS] = {
	// Transition from Start to Configure
	{.startStateIndex	= S_START,
	 .endStateIndex		= S_CONFIGURE,
	 .numFiredEvents	= 1,
	 .firedEventIndices = new unsigned int[1]{E_CONFIGURE_ENTERED}},
	// Transition from Configure to Idle
	{.startStateIndex	= S_CONFIGURE,
	 .endStateIndex		= S_IDLE,
	 .numFiredEvents	= 1,
	 .firedEventIndices = new unsigned int[1]{E_IDLE_ENTERED}},
	// Transition from Idle to Idle (self-loop)
	{.startStateIndex	= S_IDLE,
	 .endStateIndex		= S_IDLE,
	 .numFiredEvents	= 0,
	 .firedEventIndices = nullptr},
	// Transition from Idle to Compile
	{.startStateIndex	= S_IDLE,
	 .endStateIndex		= S_COMPILE,
	 .numFiredEvents	= 1,
	 .firedEventIndices = new unsigned int[1]{E_COMPILE_ENTERED}},
	// Transition from Idle to Execute
	{.startStateIndex	= S_IDLE,
	 .endStateIndex		= S_EXECUTE,
	 .numFiredEvents	= 1,
	 .firedEventIndices = new unsigned int[1]{E_EXECUTE_ENTERED}},
	// Transition from Compile to Execute
	{.startStateIndex	= S_COMPILE,
	 .endStateIndex		= S_EXECUTE,
	 .numFiredEvents	= 1,
	 .firedEventIndices = new unsigned int[1]{E_EXECUTE_ENTERED}},
	{.startStateIndex	= S_EXECUTE,
	 .endStateIndex		= S_EXECUTE,
	 .numFiredEvents	= 1,
	 .firedEventIndices = new unsigned int[1]{E_EXECUTE_ENTERED}},
};

// Define the reactions
inline struct event_reaction reactions[NUM_REACTIONS] = {
	{.numTransitions	  = 1,
	 .conditionEventIndex = E_CONFIGURE_EXIT,
	 .transitionIndices	  = new unsigned int[1]{T_CONFIGURE_IDLE}},
	{.numTransitions	  = 1,
	 .conditionEventIndex = E_IDLE_EXIT_EXECUTE,
	 .transitionIndices	  = new unsigned int[1]{T_IDLE_EXECUTE}},
	{.numTransitions	  = 1,
	 .conditionEventIndex = E_IDLE_ENTERED,
	 .transitionIndices	  = new unsigned int[1]{T_IDLE_EXECUTE}},
	{.numTransitions	  = 1,
	 .conditionEventIndex = E_IDLE_EXIT_COMPILE,
	 .transitionIndices	  = new unsigned int[1]{T_IDLE_COMPILE}},
	{.numTransitions	  = 3,
	 .conditionEventIndex = E_STEP,
	 .transitionIndices	  = new unsigned int[3]{T_START_CONFIGURE, T_IDLE_IDLE, T_EXECUTE_EXECUTE}}
};

// event data
inline struct events eventData = {
	.numEvents	   = NUM_EVENTS,
	.currentEvents = new _Bool[NUM_EVENTS]{false},
	.futureEvents  = new _Bool[NUM_EVENTS]{false},
};

// fsm
inline struct fsm_nbx fsm = {
	.numReactions	= NUM_REACTIONS,
	.numTransitions = NUM_TRANSITIONS,
	.numStates		= NUM_STATES,

	.states			   = states,
	.startStateIndex   = S_START,
	.endStateIndex	   = S_EXIT,
	.currentStateIndex = S_START,

	.eventData	 = &eventData,
	.reactions	 = reactions,
	.transitions = transitions,
};

#endif // EDDIE_ROS_FSM_HPP
