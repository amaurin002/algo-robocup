TESTS = basic_problem_1 \
				basic_problem_2 \
				goal_keeper_problem_1 \
				initial_defenders_problem \
				min_dist_problem \
				multigoal_problem

DEPOT = ../ssl_defender_viewer
CONFIGS = $(DEPOT)/configs
BUILD = ../build

all: basic_problem_2

basic_problem_1:
	python3 brut_force.py $(CONFIGS)/$@.json $(BUILD)/$@
basic_problem_2:
	python3 brut_force.py $(CONFIGS)/$@.json $(BUILD)/$@
goal_keeper_problem_1:
	python3 brut_force.py $(CONFIGS)/$@.json $(BUILD)/$@
multigoal_problem_1:
	python3 brut_force.py	$(CONFIGS)/$@.json $(BUILD)/$@

basic_problem_1_check: basic_problem_1
	python3 $(DEPOT)/main.py $(CONFIGS)/$<.json $(BUILD)/$<.json
basic_problem_2_check: basic_problem_2
	python3 $(DEPOT)/main.py $(CONFIGS)/$<.json $(BUILD)/$<.json
goal_keeper_problem_1_check: goal_keeper_problem_1
	python3 $(DEPOT)/main.py $(CONFIGS)/$<.json $(BUILD)/$<.json
multigoal_problem_1_check: multigoal_problem_1
	python3 $(DEPOT)/main.py $(CONFIGS)/$<.json $(BUILD)/$<.json


clean:
	//TODO
