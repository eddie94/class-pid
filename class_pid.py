import matplotlib.pyplot as plt

class PID:
    def __init__(self):
        #initialize gain
        self.kp=0           #proportional gain
        self.ki=0           #integral gain
        self.kd=0           #derivative gain
        self.setpoint=0     #the desired value
        self.process=0      #your current value
        self.dt=0           #the step size
        self.iteration=0    #the number of iterations

# set functions set your proportional, integral, derivative gain

    def set_kp(self,kval):
        self.kp=kval        #set your proportional gain

    def set_ki(self,ival):
        self.ki=ival        #set your integral gain

    def set_kd(self,dval):
        self.kd=dval        #set your derivative gain

#setup function sets your initial value, the desired value, step size, the number of iteration

    def setup(self,starting_point,destination,step_size,iteration_num):
        self.setpoint=destination
        self.process=starting_point
        self.dt=step_size
        self.iteration=iteration_num

#use the run function to plot your outputs

    def run(self):
        time=[]                                                 #a list that contains the x axis which is time
        measured_value_data=[]                                  #a list that contains the output value the y axis

        measured_value=0                                        #your output value
        pass_time=0                                             #how much time has been passed
        integral=0                                              #initial integral value
                                                                #has to be 0 because you didn't made a step yet
        previous_error=0                                        #also has to be 0 reason is same with 'integral'

        for i in range(self.iteration):                         #add values to the list and calculate your output
            time.append(pass_time)                              #each step you add the value of the time passed
            measured_value_data.append(measured_value)          #same as the upper one
            error=self.setpoint - measured_value                #a closed loop error is R(s)-Y(s) when R(s) is the desired value, and Y(s) the measured value

#in a closed loop, each step has its own gain
#the output of the proportional term has to be kp*R(s)
#the integral term has to be (ki/s)*R(s)
#the derivative term has to be s*kd*R(s)

            integral=integral+error*self.dt                     # discrete integral
            derivative=(error-previous_error)/self.dt           #discrete differentiation
            proportional=self.kp * error

            output = self.kp * proportional + self.ki * integral + self.kd * derivative         #add all the terms as an output
            previous_error=error                                #reset your current error
            measured_value+=output                              #update your measured value
            pass_time+=self.dt                                  #update your x axis

        plt.plot(time,measured_value_data)
        plt.show()