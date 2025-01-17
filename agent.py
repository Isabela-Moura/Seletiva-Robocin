from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Geometry import Geometry
from utils.Point import Point

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.attraction_constant = 5.0 
        self.repulsion_constant = 0.30
        self.repulsion_radius = 0.60

    def decision(self):
        if len(self.targets) == 0:
            return
        
        # Descobre o teammate mais proximo de cada alvo 
        # E atribui a cada alvo o teammate mais próximo
        target_assigments = {target: None for target in self.targets}
        for target in self.targets:
            closest_teammate_id = None
            closest_distance = float('inf')

            for teammate_id, teammate in self.teammates.items():
                distance = Geometry.dist_to(target, teammate)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_teammate_id = teammate_id
                
            target_assigments[target] = closest_teammate_id

        # Desvia dos obstaculos 
        # Utilizando o algoritmo de Campos Potenciais Artificiais
        for target, assigned_id in target_assigments.items():
            if assigned_id == self.id:
                agent_pos = self.pos

                atraction = self.calculate_attraction(agent_pos, target)
                repulsion = self.calculate_repulsion(agent_pos)

                total_force = (
                    atraction[0] + repulsion[0],
                    atraction[1] + repulsion[1]
                )

                normalized_force = self.normalize_vector(total_force)

                new_pos = Point(
                    agent_pos.x + normalized_force[0] * 0.75,
                    agent_pos.y + normalized_force[1] * 0.75
                )

                
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, new_pos)
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)
        return

    def post_decision(self):
        pass

    # Funcoes necessarias para implementar o metodo de Campos Potenciais Artificiais
    def normalize_vector(self, vector):
        magnitude = (vector[0]**2 + vector[1]**2)**0.5  
        if magnitude == 0: 
            return (0, 0)
        return (vector[0] / magnitude, vector[1] / magnitude)
    
    def direction_to(self, origin, destination):
        dx = destination.x - origin.x
        dy = destination.y - origin.y
        magnitude = (dx**2 + dy**2)**0.5  
        if magnitude == 0: 
            return (0, 0)
    
        return (dx / magnitude, dy / magnitude)
    
    def calculate_attraction(self, agent_pos, target): 
        distance = Geometry.dist_to(agent_pos, target)
        direction = self.direction_to(agent_pos, target)
        force_magnitude = self.attraction_constant * distance
        
        return (force_magnitude * direction[0], force_magnitude * direction[1])
    
    def calculate_repulsion(self, agent_pos):
        total_repulsion_force = [0, 0]
        
        self.obstacles = {**self.teammates, **self.opponents}
     
        for obstacle_id, obstacle in self.obstacles.items():
            distance = Geometry.dist_to(agent_pos, obstacle)
            if distance < self.repulsion_radius and distance != 0: 
                direction = self.direction_to(obstacle, agent_pos)
                force_magnitude = self.repulsion_constant * (1 / distance - 1 / self.repulsion_radius) / (distance ** 2)
                total_repulsion_force[0] += force_magnitude * direction[0]
                total_repulsion_force[1] += force_magnitude * direction[1]
            elif distance == 0:
                total_repulsion_force[0] += self.repulsion_constant
                total_repulsion_force[1] += self.repulsion_constant

        return total_repulsion_force


    
