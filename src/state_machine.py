class State:
    def run(self):
        pass
    def next(self, input):
        pass

class WaitingForRequest(State):
    def run(self):
        while not shutdown:
            if UserInput:
                GraspMachine.planning_grasp.run()
                continue


    def next(self, input):
        if input == Transitions.Error:
            return GraspMachine.error_state
        elif input == Transitions.RequestRecieved:
            return GraspMachine.planning_grasp
        return GraspMachine.waiting_for_request

class PlanningGrasp(State):
    def run(self):
        print("getting grasp point position of specified object")
        print("get pre grasp position")
        #get grasp point
    def next

class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    def advance_state(self):
        self.currentState = self.currentState.next()
        self.currentState.run()

class GraspMachine(StateMachine):
    waiting_for_request = WaitingForRequest()
    planning_grasp = PlanningGrasp()
    def __init__(self):
        StateMachine.__init__(self, GraspMachine.waiting_for_request)
        self.currentState.run()

