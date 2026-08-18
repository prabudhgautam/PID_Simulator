
# PID controller with back-calculation anti-windup mechanism

import matplotlib.pyplot as plt

class controller:
    def __init__(self):
        self.Kp = 0.5
        self.ki = 0.2
        self.kd = 0.25
        self.kb = 1.0
        self.raw_power = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
    def update(self, desired_velocity, MOTOR):
        self.error = desired_velocity - MOTOR.current_velocity
        self.derivative = (self.error- self.previous_error)/MOTOR.dt
        self.P = self.Kp * self.error
        self.I = self.ki*self.integral
        self.D = self.kd * self.derivative
        self.raw_power = self.P + self.I + self.D
        self.power = max(min(self.raw_power, 100),-100)
        self.integral += (self.error + self.kb*(self.power - self.raw_power))*MOTOR.dt
        self.previous_error = self.error
        return self.power
                       #Inew​=Iold​+(e+Kaw​(uact​−uPID​))dt
class motor:
    def __init__(self):
        self.friction = 0.2
        self.inertia = 0.5
        self.current_velocity = 0.0
        self.dt = 0.1
    def update(self, power, disturbance):
          self.acceleration = (power - self.friction * self.current_velocity - disturbance) / self.inertia
          self.current_velocity += self.acceleration * self.dt 
          return self.current_velocity

class input:
    def update(self,i):
                if i<40:
                    self.disturbance = 0
                else:
                    self.disturbance = 20
                if i<50:
                    self.desired_velocity = 600
                else:
                    self.desired_velocity = 60
                return self.desired_velocity,self.disturbance
class readings:
    def review(self,i,CONTROLLER,MOTOR,INPUT,velocity):
         print(
              f"Time:{i*MOTOR.dt:.1f} "
              f"Error:{CONTROLLER.error:.2f} "
              f"raw_power:{CONTROLLER.raw_power:.1f} "
              f"Power:{CONTROLLER.power:.1f} "
              f"Velocity:{velocity:.1f} "
              f"acceleration:{MOTOR.acceleration:.1f} "
              f"derivative:{CONTROLLER.derivative:.1f} "
              f"disturbance:{INPUT.disturbance} "
              f"P={CONTROLLER.P:.2f} "
              f"I={CONTROLLER.I:.2f} "
              f"D={CONTROLLER.D:.2f} ")
    def final_review(self,velocities, desired_velocities):
        self.index = None
        for i in range(1, len(desired_velocities)):
            if desired_velocities[i] != desired_velocities[i-1]:
                self.index = i
                break
        self.first_overshoot = max(0, max(velocities[:self.index]) - desired_velocities[0])
        self.second_overshoot = max(0, max(velocities[self.index:]) - desired_velocities[self.index])
        self.before_step_change_error = desired_velocities[0] - velocities[self.index-1]
        self.simulation_end_error = desired_velocities[self.index] - velocities[-1]
        print("Overshoot1:", self.first_overshoot)
        print("Overshoot2:", self.second_overshoot)
        print("error before step change:", self.before_step_change_error)
        print("Final error:", self.simulation_end_error)
if __name__ == "__main__":
    INPUT = input()
    CONTROLLER = controller()
    MOTOR = motor()
    READINGS = readings()
    velocities = []
    time = []
    desired_velocities = []
    for i in range(200):
        desired_velocity,disturbance = INPUT.update(i)
        power = CONTROLLER.update(desired_velocity, MOTOR)
        velocity = MOTOR.update(power, disturbance)
        velocities.append(velocity)
        desired_velocities.append(desired_velocity)
        time.append(i * MOTOR.dt)
        READINGS.review(i, CONTROLLER, MOTOR, INPUT, velocity)
    READINGS.final_review(velocities,desired_velocities)
    plt.plot(time, velocities, 'o-')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.grid(True)
    plt.show()
