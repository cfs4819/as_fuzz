import random
from deap import base, creator, tools, algorithms

from gene import *


class CEGA:
    def __init__(self,
                 logger=None):

        self.max_generation = 100
        self.num_individuals = 10

        self.logger = logger

        self.scene_length = 30
        self.scene_width = 30
        pass

    def evaluate(self,):
        pass

    def mate_walkers(self, ind1: GeneNpcWalkerList, ind2: GeneNpcWalkerList):
        pass

    def mate_vehicles(self, ind1: GeneNpcVehicleList, ind2: GeneNpcVehicleList):
        pass

    def mutate_walkers(self, ind: GeneNpcWalkerList):
        mut_pb = random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return ind

        # add a random vehicle, p = 0.3
        elif mut_pb <= 0.2 + 0.3:
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

    def mutate_vehicles(self, ind: GeneNpcVehicleList):
        mut_pb = random()

        # remove a random agent, p = 0.2
        if mut_pb <= 0.2 and len(ind.list) > 1:
            ind.list.remove(random.choice(ind.list))
            return ind

        # add a random agent, p = 0.3

        # mutate a random agent, p = 0.5

        pass

    def main_progress(self):
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
