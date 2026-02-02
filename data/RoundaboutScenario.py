import numpy as np
import json
import math
import matplotlib.pyplot as plt

class RoundaboutCollisionGenerator:
    def __init__(self):
        # --- 场景参数 ---
        self.radius = 20.0
        self.center = (0, 0)
        self.v_target_kmh = 30.0
        self.ttc = 3.7
        self.collision_angle_deg = 270 
        self.sim_duration = self.ttc + 3.0 
        self.sample_rate = 0.1

    def calculate_trajectory(self):
        v_ms = self.v_target_kmh / 3.6
        angular_velocity = v_ms / self.radius
        
        # 倒推起点
        angle_travelled_rad = angular_velocity * self.ttc
        start_rad = math.radians(self.collision_angle_deg) - angle_travelled_rad
        
        trajectory_points = []
        times = np.arange(0, self.sim_duration, self.sample_rate)
        
        for t in times:
            current_rad = start_rad + angular_velocity * t
            x = self.center[0] + self.radius * math.cos(current_rad)
            y = self.center[1] + self.radius * math.sin(current_rad)
            # 修正朝向：OpenSCENARIO里 0度通常朝东(x+)，90度朝北(y+)
            heading_rad = current_rad + (math.pi / 2)
            
            point = {
                "time": round(float(t), 3),
                "x": round(x, 4),
                "y": round(y, 4),
                "h": round(heading_rad, 4)
            }
            trajectory_points.append(point)
        return trajectory_points

    def generate_full_xosc(self, data, filename="MyRoundabout.xosc"):
        # 计算碰撞点(白车位置)
        coll_rad = math.radians(self.collision_angle_deg)
        ego_x = self.center[0] + self.radius * math.cos(coll_rad)
        ego_y = self.center[1] + self.radius * math.sin(coll_rad)
        ego_h = coll_rad + math.pi 

        # 1. 生成轨迹XML
        # 【修改点A】：删除了 z="0.0"，让车自动贴地
        vertex_str = ""
        for p in data:
            vertex_str += f"""
            <Vertex time="{p['time']}">
                <Position>
                    <WorldPosition x="{p['x']}" y="{p['y']}" h="{p['h']}"/>
                </Position>
            </Vertex>"""

        # 2. 生成XOSC内容
        # 【修改点B】：revMinor="1" (升级到 OpenSCENARIO 1.1 以支持 Vertex time)
        # 【修改点C】：Init阶段增加了SpeedAction，防止车辆休眠
        xosc_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
    <FileHeader revMajor="1" revMinor="1" date="2024-01-20T12:00:00" description="Roundabout" author="PyGen"/>
    <ParameterDeclarations/>
    <CatalogLocations>
        <VehicleCatalog>
            <Directory path="Catalogs/Vehicles"/>
        </VehicleCatalog>
    </CatalogLocations>
    <RoadNetwork>
        <LogicFile filepath=""/>
    </RoadNetwork>
    
    <Entities>
        <ScenarioObject name="TargetCar">
            <CatalogReference catalogName="VehicleCatalog" entryName="car_red"/>
        </ScenarioObject>
        <ScenarioObject name="Ego">
            <CatalogReference catalogName="VehicleCatalog" entryName="car_white"/>
        </ScenarioObject>
    </Entities>

    <Storyboard>
        <Init>
            <Actions>
                <Private entityRef="TargetCar">
                    <PrivateAction>
                        <TeleportAction>
                            <Position>
                                <WorldPosition x="{data[0]['x']}" y="{data[0]['y']}" h="{data[0]['h']}"/>
                            </Position>
                        </TeleportAction>
                    </PrivateAction>
                    <PrivateAction>
                        <LongitudinalAction>
                            <SpeedAction>
                                <SpeedActionDynamics dynamicsShape="step" value="0.0" dynamicsDimension="time"/>
                                <SpeedActionTarget>
                                    <AbsoluteTargetSpeed value="{self.v_target_kmh / 3.6}"/>
                                </SpeedActionTarget>
                            </SpeedAction>
                        </LongitudinalAction>
                    </PrivateAction>
                </Private>
                <Private entityRef="Ego">
                    <PrivateAction>
                        <TeleportAction>
                            <Position>
                                <WorldPosition x="{ego_x}" y="{ego_y}" h="{ego_h}"/>
                            </Position>
                        </TeleportAction>
                    </PrivateAction>
                </Private>
            </Actions>
        </Init>
        <Story name="MainStory">
            <Act name="MoveAct">
                <ManeuverGroup maximumExecutionCount="1" name="MoveSeq">
                    <Actors selectTriggeringEntities="false">
                        <EntityRef entityRef="TargetCar"/>
                    </Actors>
                    <Maneuver name="MoveManeuver">
                        <Event name="MoveEvent" priority="overwrite">
                            <Action name="MoveAction">
                                <PrivateAction>
                                    <RoutingAction>
                                        <FollowTrajectoryAction>
                                            <Trajectory name="RoundaboutTraj" closed="false">
                                                <Shape>
                                                    <Polyline>{vertex_str}</Polyline>
                                                </Shape>
                                            </Trajectory>
                                            <TimeReference>
                                                <Timing domainAbsoluteRelative="absolute" scale="1.0" offset="0.0"/>
                                            </TimeReference>
                                            <TrajectoryFollowingMode followingMode="position"/>
                                        </FollowTrajectoryAction>
                                    </RoutingAction>
                                </PrivateAction>
                            </Action>
                            <StartTrigger>
                                <ConditionGroup>
                                    <Condition name="Start" delay="0" conditionEdge="rising">
                                        <ByValueCondition>
                                            <SimulationTimeCondition value="0" rule="greaterThan"/>
                                        </ByValueCondition>
                                    </Condition>
                                </ConditionGroup>
                            </StartTrigger>
                        </Event>
                    </Maneuver>
                </ManeuverGroup>
                <StartTrigger>
                    <ConditionGroup>
                        <Condition name="ActStart" delay="0" conditionEdge="rising">
                            <ByValueCondition>
                                <SimulationTimeCondition value="0" rule="greaterThan"/>
                            </ByValueCondition>
                        </Condition>
                    </ConditionGroup>
                </StartTrigger>
            </Act>
        </Story>
        <StopTrigger/>
    </Storyboard>
</OpenSCENARIO>
"""
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(xosc_content)
        print(f"✅ 终极修复版文件已生成: {filename}")

if __name__ == "__main__":
    generator = RoundaboutCollisionGenerator()
    traj = generator.calculate_trajectory()
    generator.generate_full_xosc(traj, "MyRoundabout.xosc")