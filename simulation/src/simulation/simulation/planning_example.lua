function setGripperData(open,velocity,force)
    if not velocity then
        velocity=0.11
    end
    if not force then
        force=20
    end
    if not open then
        velocity=-velocity
    end
    
    local dat={}
    dat.velocity=velocity
    dat.force=force
    sim.setBufferProperty(params.gripper, 'customData.activity',sim.packTable(dat))
end

function moveToPose(pose)
    local p = {
        ik = {tip = params.robotTip, target = params.robotTarget, base = params.robotBase, joints = params.joints},
        targetPose = pose,
        maxVel = params.ikMaxVel,
        maxAccel = params.ikMaxAccel,
        maxJerk = params.ikMaxJerk,
    }
    sim.moveToPose(p)
end

function collides(configs)
    -- checks if the configs are related to a collision with the environment or self-collision
    local retVal = false
    local bufferedConfig = getConfig()
    for i = 1, #configs do
        setConfig(configs[i])
        local res = sim.checkCollision(params.robotCollection, sim.handle_all)
        if res > 0 then
            retVal = true
            break
        else
            res = sim.checkCollision(params.robotCollection, params.robotCollection)
            if res > 0 then
                retVal = true
                break
            end
        end
    end
    setConfig(bufferedConfig)
    return retVal
end

function selectOneValidConfig(configs, approachIkTr, withdrawIkTr)
    local retVal, passiveVizShape
    local bufferedConfig = getConfig()
    for i = 1, #configs do
        local target = configs[i]
        if not collides({target}) then
            setConfig(target)
            -- if there is an approach ik transformation, check if it is valid:
            if approachIkTr then
                local pose = sim.getObjectPose(params.robotTip)
                local targetPose = sim.multiplyPoses(pose, approachIkTr)
                sim.setObjectPose(params.robotTarget, targetPose)
                local ikEnv = simIK.createEnvironment()
                local ikGroup = simIK.createGroup(ikEnv)
                local ikEl, simToIk, ikToSim = simIK.addElementFromScene(ikEnv, ikGroup, params.robotBase, params.robotTip, params.robotTarget, simIK.constraint_pose)
                local ikJoints={}
                for i=1,6,1 do
                    ikJoints[i] = simToIk[params.joints[i]]
                end
                local path = simIK.generatePath(ikEnv, ikGroup, ikJoints, simToIk[params.robotTip], 4)
                simIK.eraseEnvironment(ikEnv)
                if path then
                    path = Matrix(#path // 6, 6, path):totable()
                    if collides(path) then
                        target = nil
                    else
                        -- if there is a withdraw ik transformation (which obviously should then be different from opposite of approach tr), check if it is valid:
                        if withdrawIkTr then
                            targetPose = sim.multiplyPoses(targetPose, withdrawIkTr)
                            sim.setObjectPose(params.robotTarget, targetPose)
                            local ikEnv = simIK.createEnvironment()
                            local ikGroup = simIK.createGroup(ikEnv)
                            local ikEl, simToIk, ikToSim = simIK.addElementFromScene(ikEnv, ikGroup, params.robotBase, params.robotTip, params.robotTarget, simIK.constraint_pose)
                            local ikJoints={}
                            for i=1,6,1 do
                                ikJoints[i] = simToIk[params.joints[i]]
                            end
                            local path = simIK.generatePath(ikEnv, ikGroup, ikJoints, simToIk[params.robotTip], 4)
                            simIK.eraseEnvironment(ikEnv)
                            if path then
                                path = Matrix(#path // 6, 6, path):totable()
                                if collides(path) then
                                    target = nil
                                end
                            else
                                target = nil
                            end
                        end
                    end
                else
                    target = nil
                end
            end
        
            if target then
                retVal = target
                local list = sim.getObjectsInTree(params.robotBase, sim.sceneobject_shape)
                local lc = table.clone(list)
                list = {}
                for j = 1, #lc do
                    if sim.getBoolProperty(lc[j], 'visible') then
                        list[#list + 1] = lc[j]
                    end
                end
                list = sim.copyPasteObjects(list)
                passiveVizShape = sim.groupShapes(list, true)
                sim.setBoolProperty(passiveVizShape, 'respondable', false)
                sim.setBoolProperty(passiveVizShape, 'dynamic', false)
                sim.setBoolProperty(passiveVizShape, 'collidable', false)
                sim.setBoolProperty(passiveVizShape, 'measurable', false)
                sim.setBoolProperty(passiveVizShape, 'detectable', false)
                sim.setColorProperty(sim.getIntArrayProperty(passiveVizShape, 'meshes')[1], 'color.diffuse', {1, 0, 0})
                sim.setObjectAlias(passiveVizShape, 'passiveVisualizationShape')
                break
            end
        end
    end
    setConfig(bufferedConfig)
    return retVal, passiveVizShape
end

function setConfig(c)
    for i = 1, #params.joints do
        sim.setJointPosition(params.joints[i], c[i])
    end
end

function setTargetConfig(c)
    for i = 1, #params.joints do
        sim.setJointTargetPosition(params.joints[i], c[i])
    end
end

function getConfig()
    local c = {}
    for i = 1, #params.joints do
        c[i] = sim.getJointPosition(params.joints[i])
    end
    return c
end

function findConfigs(pose)
    local ikEnv = simIK.createEnvironment()
    local ikGroup = simIK.createGroup(ikEnv)
    local ikEl, simToIk, ikToSim = simIK.addElementFromScene(ikEnv, ikGroup, params.robotBase, params.robotTip, params.robotTarget, simIK.constraint_pose)
    local ikJoints={}
    for i=1,6,1 do
        ikJoints[i] = simToIk[params.joints[i]]
    end
    sim.setObjectPose(params.robotTarget, pose)
    simIK.syncFromSim(ikEnv, {ikGroup}) -- make sure the arm is in the same configuration in the IK world!
    local p = {}
    p.maxDist = 0.28
    p.maxTime = 1
    p.cMetric = {8, 8, 8, 0.8, 0.6, 0.3}
    
    p.findMultiple = true
    local retVal = simIK.findConfigs(ikEnv, ikGroup, ikJoints, p)
    simIK.eraseEnvironment(ikEnv)
    return retVal
end

function findPath(config)
    local useForProjection = {}
    for i = 1, #params.joints do
        useForProjection[i] = (i <= 3 and 1 or 0)
    end
    local retVal
    local task = simOMPL.createTask('task')
    simOMPL.setAlgorithm(task, params.pathPlanningAlgo)
    simOMPL.setStateSpaceForJoints(task, params.joints, useForProjection)
    simOMPL.setCollisionPairs(task, {params.robotCollection, sim.handle_all, params.robotCollection, params.robotCollection})
    simOMPL.setStartState(task, getConfig())
    simOMPL.setGoalState(task, config)
    -- simOMPL.addGoalState
    simOMPL.setup(task)
    
    if simOMPL.solve(task, params.pathPlanningMaxTime) and simOMPL.hasExactSolution(task) then
        simOMPL.simplifyPath(task, params.pathPlanningMaxSimplificationTime)
        retVal = simOMPL.getPath(task)
    end
    simOMPL.destroyTask(task)
    
    return retVal
end

function followPath(path)
    local minMaxVel = {-params.fkMaxVel[1], params.fkMaxVel[1], -params.fkMaxVel[2], params.fkMaxVel[2], -params.fkMaxVel[3], params.fkMaxVel[3], -params.fkMaxVel[4], params.fkMaxVel[4], -params.fkMaxVel[5], params.fkMaxVel[5], -params.fkMaxVel[6], params.fkMaxVel[6]}
    local minMaxAccel = {-params.fkMaxAccel[1], params.fkMaxAccel[1], -params.fkMaxAccel[2], params.fkMaxAccel[2], -params.fkMaxAccel[3], params.fkMaxAccel[3], -params.fkMaxAccel[4], params.fkMaxAccel[4], -params.fkMaxAccel[5], params.fkMaxAccel[5], -params.fkMaxAccel[6], params.fkMaxAccel[6]}
    local pl = sim.getPathLengths(path, 6)
    if followPathScript == nil then
        followPathScript = -1 -- recycle this script in next calls!
    end
    local pathPts, times
    pathPts, times, followPathScript = sim.generateTimeOptimalTrajectory(path, pl, minMaxVel, minMaxAccel, 1000, 'not-a-knot', 5, followPathScript)
    
    local st = sim.getSimulationTime()
    local dt = 0
    while dt < times[#times] do
        local p = sim.getPathInterpolatedConfig(pathPts, times, dt)
        setTargetConfig(p)
        sim.step()
        dt = sim.getSimulationTime() - st
    end
    local p = sim.getPathInterpolatedConfig(pathPts, times, times[#times])
    setTargetConfig(p)
end

function sysCall_thread()
    sim=require'sim'
    simIK=require'simIK'
    simOMPL=require'simOMPL'
    sim.setStepping(true)
    
    -- Prepare some values/parameters:
    params = {}
    params.joints = {}
    for i = 1, 6 do
        params.joints[i] = sim.getObject('../joint', {index = i - 1})
    end
    params.gripperSensor = sim.getObject('../attachProxSensor')
    params.robotTip=sim.getObject('../tip')
    params.robotTarget=sim.getObject('../target')
    params.robotBase=sim.getObject('..')
    params.gripper=sim.getObject('../RG2')
    params.conveyorSensor=sim.getObject('/conveyor[0]/_sensor')
    params.robotCollection = sim.createCollection()
    sim.addItemToCollection(params.robotCollection, sim.handle_tree, params.robotBase, 0)
    params.pathPlanningMaxTime = 4.0
    params.pathPlanningMaxSimplificationTime = 4.0
    params.pathPlanningAlgo = simOMPL.Algorithm.RRTstar
    
    local dropPoses={}
    for i = 1, 4 do
        dropPoses[i] = sim.getObjectPose(sim.getObject('/dropPose' .. i))
    end

    -- IK motions:
    params.ikMaxVel = {0.4, 0.4, 0.4, 1.8}
    params.ikMaxAccel = {0.8, 0.8, 0.8, 0.9}
    params.ikMaxJerk = {0.6, 0.6, 0.6, 0.8}

    -- FK motions:
    local fkVel = 180
    local fkAccel = 40
    local fkJerk = 80
    params.fkMaxVel = {fkVel * math.pi / 180, fkVel * math.pi / 180, fkVel * math.pi / 180, fkVel * math.pi / 180, fkVel * math.pi / 180, fkVel * math.pi / 180}
    params.fkMaxAccel = {fkAccel * math.pi / 180, fkAccel * math.pi / 180, fkAccel * math.pi / 180, fkAccel * math.pi / 180, fkAccel * math.pi / 180, fkAccel * math.pi / 180}
    params.fkMaxJerk = {fkJerk * math.pi / 180, fkJerk * math.pi / 180, fkJerk * math.pi / 180, fkJerk * math.pi / 180, fkJerk * math.pi / 180, fkJerk * math.pi / 180}

    local initConf = getConfig()
    local dropIndex = 1
    local pickPose = sim.getObjectPose(sim.getObject('/pickPose'))
    local droppedPartsCnt = 0
    setGripperData(true)
    sim.wait(1.0) -- wait a bit until the gripper is fully open
    while sim.checkProximitySensor(params.conveyorSensor, sim.handle_all) == 1 do
        local cube = pickPart(pickPose, {-0.105, 0, 0, 0, 0, 0, 1}, {0.105, 0, -0.1, 0, 0, 0, 1})
        local dropPose = dropPoses[dropIndex]
        local pose = table.clone(dropPose)
        local dropApproachIkTr = {-0.18 + 0.05 * (droppedPartsCnt // 2), 0, 0, 0, 0, 0, 1}
        dropPart(cube, pose, dropApproachIkTr)
        dropIndex = dropIndex + 1
        if dropIndex > 4 then
            dropIndex = 1
        end
        droppedPartsCnt = droppedPartsCnt + 1
    end

    local pathBackHome = findPath(initConf)
    if pathBackHome then
        print('Found a path from the current config back to the home config!')
        followPath(pathBackHome)
    else
        error('Failed finding a path from the current config back to the home config. Try increasing the search times.')
    end
    sim.stopSimulation()
end

function pickPart(pickPose, approachIkTr, withdrawIkTr)
    local configs = findConfigs(pickPose)
    if #configs > 0 then
        print(string.format('Found %i different configs corresponding to the desired pick pose. Now selecting an appropriate valid config...', #configs))
        local pickConfig, passiveVizShape = selectOneValidConfig(configs, approachIkTr, withdrawIkTr) -- select the closest config to current config
        sim.step() -- to make the passiveVizShape immediately visible
        print('Selected following pick config: ', (Vector(pickConfig) * 180.0 / math.pi):data())
        
        local path = findPath(pickConfig)
        if path then
            print('Found a path from the current config to the pick config!')
            followPath(path)
            if passiveVizShape then
                sim.removeObjects({passiveVizShape})
            end
        else
            error('Failed finding a path from the current config to the pick config. Try increasing the search times.')
        end

        local pose = sim.getObjectPose(params.robotTip)
        pose = sim.multiplyPoses(pose, approachIkTr)
        moveToPose(pose) -- move towards object to pick via IK
        setGripperData(false)
        sim.wait(0.5)
        local res, d, pt, cube = sim.checkProximitySensor(params.gripperSensor, sim.handle_all)
        if cube and cube >= 0 then
            sim.setObjectParent(cube, params.gripper)
            sim.setIntProperty(cube, 'collectionSelfCollisionIndicator', 10) -- so that the cube doesn't generate a robot-self collision between gripper parts and cube
        end
        pose = sim.getObjectPose(params.robotTip)
        pose = sim.multiplyPoses(pose, withdrawIkTr)
        moveToPose(pose) -- move back and lift object via IK
        return cube
    else
        error('Failed finding a config corresponding to the desired pick pose.')
    end
end

function dropPart(cube, dropPose, approachIkTr)
    local configs = findConfigs(dropPose)
    if #configs > 0 then
        print(string.format('Found %i different configs corresponding to the desired drop pose. Now selecting an appropriate valid config...', #configs))
        local dropConfig, passiveVizShape = selectOneValidConfig(configs, approachIkTr) -- select the closest config to current config
        sim.step() -- to make the passiveVizShape immediately visible
        print('Selected following drop config: ', (Vector(dropConfig) * 180.0 / math.pi):data())

        local path = findPath(dropConfig)
        if path then
            print('Found a path from the current config to the drop config!')
            followPath(path)
            if passiveVizShape then
                sim.removeObjects({passiveVizShape})
            end
        else
            error('Failed finding a path from the current config to the drop config. Try increasing the search times.')
        end

        local pose = sim.getObjectPose(params.robotTip)
        pose = sim.multiplyPoses(pose, approachIkTr)
        moveToPose(pose) -- move towards drop location via IK
        setGripperData(true)
        sim.wait(0.5)
        moveToPose(dropPose) -- move back up again via IK
        
        if cube and cube >= 0 then
            sim.setObjectParent(cube, -1)
        end
        sim.wait(0.5)
    else
        error('Failed finding a config corresponding to the desired drop pose.')
    end
end