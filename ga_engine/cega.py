import random
from deap import base, creator, tools, algorithms

from gene import *
from copy import deepcopy
from MS_fuzz.fuzz_config.Config import Config

class CEGA:
    def __init__(self,
                 cfgs:Config,
                 logger=None):

        self.max_generation = 100
        self.num_individuals = 10

        self.logger = logger

        self.scene_length = cfgs.scenario_length
        self.scene_width = cfgs.scenario_width

    def evaluate(self,):
        pass

    def mate_walkers(self, ind1: GeneNpcWalkerList, ind2: GeneNpcWalkerList):
        offspring = GeneNpcWalkerList

        for index in range(min(len(ind1.list), len(ind2.list))):
            parent1 = ind1.list[index]
            parent2 = ind2.list[index]

            walker = GeneNpcWalker()
            walker.start = random.choice([parent1.start, parent2.start])
            walker.end = random.choice([parent1.end, parent2.end])
            walker.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            walker.status = random.choice([parent1.status, parent2.status])
            walker.max_speed = random.choice([parent1.max_speed, parent2.max_speed])
            
            offspring.list.append(walker)

        for index in range(min(len(ind1.list), len(ind2.list)),
                           max(len(ind1.list), len(ind2.list))):
            if len(ind1.list) > len(ind2.list):
                offspring.list.append(deepcopy(ind1.list[index]))
            else:
                offspring.list.append(deepcopy(ind2.list[index]))
        
        return offspring

    def mate_vehicles(self, ind1: GeneNpcVehicleList, ind2: GeneNpcVehicleList):
        offspring = GeneNpcVehicleList()

        for index in range(min(len(ind1.list), len(ind2.list))):
            parent1 = ind1.list[index]
            parent2 = ind2.list[index]

            vehicle = GeneNpcVehicle()

            vehicle.start = random.choice([parent1.start, parent2.start])
            vehicle.end = random.choice([parent1.end, parent2.end])
            vehicle.start_time = random.choice(
                [parent1.start_time, parent2.start_time])
            vehicle.agent_type = random.choice(
                [parent1.agent_type, parent2.agent_type])
            vehicle.status = random.choice([parent1.status, parent2.status])

            if vehicle.status == 0:
                vehicle.initial_speed = random.choice(
                    [parent1.initial_speed, parent2.initial_speed])

            offspring.list.append(vehicle)

        for index in range(min(len(ind1.list), len(ind2.list)),
                           max(len(ind1.list), len(ind2.list))):
            if len(ind1.list) > len(ind2.list):
                offspring.list.append(deepcopy(ind1.list[index]))
            else:
                offspring.list.append(deepcopy(ind2.list[index]))

        return offspring

    def mutate_walkers(self, ind: GeneNpcWalkerList):
        mut_pb = random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return ind

        # add a random agent, p = 0.3
        elif ((mut_pb <= 0.2 + 0.3 and len(ind.list) < ind.max_walker_count) or
              len(ind.list) < 1):
            # generate a random start position and end position
            start_x = random.uniform(-self.scene_length/2, self.scene_length/2)
            start_y = random.uniform(-self.scene_width/2, self.scene_width/2)
            end_x = random.uniform(-self.scene_length/2, self.scene_length/2*3)
            end_y = random.uniform(-self.scene_width/2, self.scene_width/2)

            new_walker = GeneNpcWalker()
            new_walker.start = {'x': start_x, 'y': start_y, 'z': 0}
            new_walker.end = {'x': end_x, 'y': end_y, 'z': 0}

            new_walker.start_time = random.uniform(0, 2)

            new_walker.status = random.choices([0, 1],
                                               weights=[0.7, 0.3], k=1)[0]

            if new_walker.status == 0:
                new_walker.max_speed = random.uniform(0, 3)

            ind.list.append(new_walker)

            return ind

        # mutate a random agent, p = 0.5
        else:
            ind.list.remove(random.choice(ind.list))
            start_x = random.uniform(-self.scene_length/2, self.scene_length/2)
            start_y = random.uniform(-self.scene_width/2, self.scene_width/2)
            end_x = random.uniform(-self.scene_length/2, self.scene_length/2*3)
            end_y = random.uniform(-self.scene_width/2, self.scene_width/2)

            new_walker = GeneNpcWalker()
            new_walker.start = {'x': start_x, 'y': start_y, 'z': 0}
            new_walker.end = {'x': end_x, 'y': end_y, 'z': 0}

            new_walker.start_time = random.uniform(0, 2)

            new_walker.status = random.choices([0, 1],
                                               weights=[0.7, 0.3], k=1)[0]

            if new_walker.status == 0:
                new_walker.max_speed = random.uniform(0, 3)

            ind.list.append(new_walker)
            return ind

    def mutate_vehicles(self, ind: GeneNpcVehicleList):
        mut_pb = random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return ind

        # add a random vehicle, p = 0.3
        elif ((mut_pb <= 0.2 + 0.3 and len(ind.list) < ind.max_walker_count) or
              len(ind.list) < 1):
            # generate a random start position and end position
            start_x = random.uniform(-self.scene_length/2, self.scene_length/2)
            start_y = random.uniform(-self.scene_width/2, self.scene_width/2)
            end_x = random.uniform(-self.scene_length/2, self.scene_length/2*3)
            end_y = random.uniform(-self.scene_width/2, self.scene_width/2)
            while abs(start_y - end_y) <= 5:
                end_y = random.uniform(-self.scene_width/2, self.scene_width/2)

            new_vehicle = GeneNpcVehicle()

            new_vehicle.start = {'x': start_x, 'y': start_y, 'z': 0}
            new_vehicle.end = {'x': end_x, 'y': end_y, 'z': 0}

            new_vehicle.start_time = random.uniform(0, 2)

            new_vehicle.vehicle_type = random.choices([0, 1, 2, 3],
                                                      weights=[0.4, 0.3, 0.2, 0.1], k=1)[0]

            new_vehicle.status = random.choices([0, 1, 2],
                                                weights=[0.6, 0.3, 0.1], k=1)[0]

            new_vehicle.agent_type = random.choices([0, 1, 2],
                                                    weights=[0.6, 0.2, 0.2], k=1)[0]
            if new_vehicle.status == 0:
                new_vehicle.initial_speed = random.uniform(0, 20)

            ind.list.append(new_vehicle)
            return ind

        # mutate a random agent, p = 0.5
        else:
            ind_2_mutate = random.choice(ind.list)
            parameters_2_mutate_list = ['start', 'end', 'start_time',
                                        'vehicle_type', 'initial_speed',
                                        'status', 'agent_type']
            mutete_weight = [0.2, 0.2, 0.1,
                             0.2, 0.1,
                             0.0, 0.2]
            # select 3 parameters to mutate
            parameters_2_mutate = random.choices(parameters_2_mutate_list,
                                                 weights=mutete_weight,
                                                 k=3)
            if 'start' in parameters_2_mutate:
                start_x = random.uniform(-self.scene_length/2,
                                         self.scene_length/2)
                start_y = random.uniform(-self.scene_width/2,
                                         self.scene_width/2)
                ind_2_mutate.start = {'x': start_x, 'y': start_y, 'z': 0}
            if 'end' in parameters_2_mutate:
                end_x = random.uniform(-self.scene_length/2,
                                       self.scene_length/2*3)
                end_y = random.uniform(-self.scene_width/2, self.scene_width/2)
                while abs(ind_2_mutate.start['y'] - end_y) <= 5:
                    end_y = random.uniform(-self.scene_width/2,
                                           self.scene_width/2)
                ind_2_mutate.end = {'x': end_x, 'y': end_y, 'z': 0}
            if 'start_time' in parameters_2_mutate:
                ind_2_mutate.start_time = random.uniform(0, 2)
            if 'vehicle_type' in parameters_2_mutate:
                ind_2_mutate.vehicle_type = random.choices([0, 1, 2, 3],
                                                           weights=[
                                                               0.4, 0.3, 0.2, 0.1],
                                                           k=1)[0]
            if 'status' in parameters_2_mutate:
                ind_2_mutate.status = random.choices([0, 1, 2],
                                                     weights=[0.6, 0.3, 0.1], k=1)[0]
            if 'initial_speed' in parameters_2_mutate and ind_2_mutate.status == 0:
                ind_2_mutate.initial_speed = random.uniform(0, 20)
            if 'agent_type' in parameters_2_mutate:
                ind_2_mutate.agent_type = random.choices([0, 1, 2],
                                                         weights=[0.6, 0.2, 0.2], k=1)[0]

            return ind

    def main_progress(self):
        if self.logger:
            self.logger.info("Start GA:")
        
        CXPB = 0.6
        MUTPB = 0.4

        # Co-evolutionary Genetic Algorithm
        tb_walkers = base.Toolbox()
        tb_vehicles = base.Toolbox()

        tb_walkers.register('mate', self.mate_walkers)
        tb_walkers.register('mutate', self.mutate_walkers)
        tb_walkers.register('select', tools.selNSGA2)

        tb_vehicles.register('mate', self.mate_vehicles)
        tb_vehicles.register('mutate', self.mutate_vehicles)
        tb_vehicles.register('select', tools.selNSGA2)

        # Evaluate Initial Population
        if self.logger:            
            self.logger.info(f' ====== Analyzing Initial Population ====== ')

        for gen in range(self.max_generation):
            if self.logger:
                self.logger.info(f"Generation #{gen}. Start:")

        pass


# 定义问题相关参数
NUM_INDIVIDUALS = 100
NUM_GENERATIONS = 50

# 定义适应度评估函数


def evaluate(ind1, ind2):
    # 自定义评估函数，计算两个种群的适应度
    fitness1, diversity1 = ind1.evaluate()  # 自定义的评估函数
    fitness2, diversity2 = ind2.evaluate()  # 自定义的评估函数
    return fitness1, fitness2, diversity1, diversity2


# 创建适应度类，包含优秀分和多样性分
creator.create("FitnessMulti", base.Fitness, weights=(1.0, 1.0, -1.0, -1.0))

# 创建个体类
creator.create("Individual", list, fitness=creator.FitnessMulti)

# 定义mate和mutate函数，用标准函数调用自定义函数


def mate(ind1, ind2):
    if ind1.is_from_popA and ind2.is_from_popA:
        return popA_mate(ind1, ind2)
    elif ind1.is_from_popB and ind2.is_from_popB:
        return popB_mate(ind1, ind2)
    else:
        raise ValueError("Invalid mating combination")


def mutate(ind):
    if ind.is_from_popA:
        return popA_mutate(ind)
    elif ind.is_from_popB:
        return popB_mutate(ind)
    else:
        raise ValueError("Invalid mutation operation")


# 创建toolbox对象
toolbox = base.Toolbox()

# 注册标准的mate和mutate函数到toolbox
toolbox.register("mate", mate)
toolbox.register("mutate", mutate)

# 注册选择算子
toolbox.register("select", tools.selNSGA2)  # 或者使用NSGAIII: tools.selNSGA3

# 注册生成个体和种群的函数
toolbox.register("individual", tools.initRepeat,
                 creator.Individual, (random(), random()), n=2)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# 注册评估函数
toolbox.register("evaluate", evaluate)

# 创建种群A和种群B
populationA = toolbox.population(n=NUM_INDIVIDUALS)
populationB = toolbox.population(n=NUM_INDIVIDUALS)

# 开始进化
for gen in range(NUM_GENERATIONS):
    offspringA = algorithms.varAnd(populationA, toolbox, cxpb=0.5, mutpb=0.2)
    offspringB = algorithms.varAnd(populationB, toolbox, cxpb=0.5, mutpb=0.2)

    fitsA = toolbox.map(toolbox.evaluate, offspringA)
    fitsB = toolbox.map(toolbox.evaluate, offspringB)

    for indA, fitA, indB, fitB in zip(offspringA, fitsA, offspringB, fitsB):
        indA.fitness.values = fitA
        indB.fitness.values = fitB

    populationA = toolbox.select(offspringA + populationA, k=NUM_INDIVIDUALS)
    populationB = toolbox.select(offspringB + populationB, k=NUM_INDIVIDUALS)

# 获取最优解
best_indA = tools.selBest(populationA, k=1)[0]
best_indB = tools.selBest(populationB, k=1)[0]

print("Best individual in population A:", best_indA)
print("Best individual in population B:", best_indB)
