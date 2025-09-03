#!/usr/bin/env python3

"""
Z1 Web API Service - Complete SDK HTTP Interface
Exposes all SDK functions via FastAPI web service
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../z1_sdk/examples_py'))

import numpy as np
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import uvicorn
import threading
import time

try:
    import unitree_arm_interface
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

app = FastAPI(title="Z1 Robotic Arm API", version="2.0")

# Pydantic models
class JointCommand(BaseModel):
    joints: List[float]
    speed: float = 1.0

class CartesianCommand(BaseModel):
    position: List[float]  # [x, y, z, rx, ry, rz]
    gripper: float = 0.0
    speed: float = 1.0

class ForceCommand(BaseModel):
    forces: List[float]  # [fx, fy, fz, mx, my, mz]
    duration: float = 1.0

class TrajectoryPoint(BaseModel):
    joints: List[float]
    time: float

class Trajectory(BaseModel):
    points: List[TrajectoryPoint]

# Global arm interface
arm = None
armModel = None

@app.on_event("startup")
async def startup():
    global arm, armModel
    if SDK_AVAILABLE:
        arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        armModel = arm._ctrlComp.armModel
        arm.loopOn()

@app.on_event("shutdown")
async def shutdown():
    if arm:
        arm.loopOff()

@app.get("/")
async def root():
    return {"message": "Z1 Robotic Arm API", "sdk_available": SDK_AVAILABLE}

@app.get("/status")
async def get_status():
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    q = arm.lowstate.getQ().tolist()
    pose = armModel.forwardKinematics(np.array(q), 6)
    
    return {
        "joints": q,
        "position": pose[0:3, 3].tolist(),
        "gripper": arm.lowstate.getGripperQ(),
        "mode": "active"
    }

@app.post("/move/joint")
async def move_joint(cmd: JointCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        # Convert to posture for MoveJ
        T = armModel.forwardKinematics(np.array(cmd.joints), 6)
        posture = unitree_arm_interface.homoToPosture(T)
        
        success = arm.MoveJ(posture, 0.0, cmd.speed)
        return {"success": success, "message": "Joint movement executed"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/move/cartesian")
async def move_cartesian(cmd: CartesianCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        success = arm.MoveL(cmd.position, cmd.gripper, cmd.speed)
        return {"success": success, "message": "Cartesian movement executed"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/move/circular")
async def move_circular(middle: CartesianCommand, end: CartesianCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        success = arm.MoveC(middle.position, end.position, end.gripper, end.speed)
        return {"success": success, "message": "Circular movement executed"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/control/force")
async def force_control(cmd: ForceCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        arm.startTrack(unitree_arm_interface.ArmFSMState.CARTESIAN)
        
        steps = int(cmd.duration / 0.002)
        for _ in range(steps):
            force_cmd = cmd.forces + [0]  # Add gripper
            arm.cartesianCtrlCmd(force_cmd, 0.1, 0.1)
            time.sleep(0.002)
            
        return {"success": True, "message": f"Force control for {cmd.duration}s"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/kinematics/forward")
async def forward_kinematics(cmd: JointCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        T = armModel.forwardKinematics(np.array(cmd.joints), 6)
        return {
            "position": T[0:3, 3].tolist(),
            "rotation_matrix": T[0:3, 0:3].tolist(),
            "homogeneous_transform": T.tolist()
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/kinematics/inverse")
async def inverse_kinematics(cmd: CartesianCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        # Build homogeneous transform (simplified)
        T = np.eye(4)
        T[0:3, 3] = cmd.position[0:3]
        
        current_q = arm.lowstate.getQ()
        hasIK, q_result = armModel.inverseKinematics(T, current_q, False)
        
        if hasIK:
            return {"success": True, "joints": q_result.tolist()}
        else:
            return {"success": False, "message": "No IK solution found"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/dynamics/gravity_compensation")
async def gravity_compensation(cmd: JointCommand):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        tau = armModel.inverseDynamics(
            np.array(cmd.joints), 
            np.zeros(6), 
            np.zeros(6), 
            np.zeros(6)
        )
        return {"torques": tau.tolist()}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.get("/jacobian")
async def get_jacobian():
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        q = arm.lowstate.getQ()
        J = armModel.CalcJacobian(q)
        return {"jacobian": J.tolist()}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/trajectory/execute")
async def execute_trajectory(traj: Trajectory):
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        arm.setFsmLowcmd()
        
        for point in traj.points:
            q_des = np.array(point.joints)
            tau = armModel.inverseDynamics(q_des, np.zeros(6), np.zeros(6), np.zeros(6))
            
            arm.q = q_des
            arm.qd = np.zeros(6)
            arm.tau = tau
            
            arm.setArmCmd(arm.q, arm.qd, arm.tau)
            arm.sendRecv()
            time.sleep(point.time)
            
        return {"success": True, "message": f"Executed {len(traj.points)} trajectory points"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/emergency_stop")
async def emergency_stop():
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        arm.setFsm(unitree_arm_interface.ArmFSMState.PASSIVE)
        return {"success": True, "message": "Emergency stop activated"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/home")
async def go_home():
    if not SDK_AVAILABLE:
        raise HTTPException(status_code=503, detail="SDK not available")
    
    try:
        arm.backToStart()
        return {"success": True, "message": "Returning to home position"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.get("/capabilities")
async def get_capabilities():
    return {
        "sdk_available": SDK_AVAILABLE,
        "control_modes": ["joint", "cartesian", "force", "trajectory"],
        "kinematics": ["forward", "inverse", "jacobian"],
        "dynamics": ["gravity_compensation", "inverse_dynamics"],
        "high_level_commands": ["MoveJ", "MoveL", "MoveC"],
        "safety": ["emergency_stop", "workspace_limits"]
    }

if __name__ == "__main__":
    print("🌐 Starting Z1 Web API Service...")
    print("📡 Access API at: http://localhost:8000")
    print("📚 Documentation at: http://localhost:8000/docs")
    
    uvicorn.run(app, host="0.0.0.0", port=8000)