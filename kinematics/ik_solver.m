%% CanBot - Inverse Kinematics Solver
%  EE297 Intelligent Systems Project, Maynooth University, 2024
%
%  Imports the CAD arm assembly from Onshape as a rigidBodyTree, then uses
%  MATLAB's generalizedInverseKinematics solver to find the joint angles
%  that place the gripper at a target coordinate supplied by the vision
%  system.
%
%  Requires: Robotics System Toolbox, Simscape Multibody,
%            Simscape Multibody Link (for smexportonshape).
%
%  NOTE: reconstructed from the console session in the technical report
%  appendix. Sections marked INFERRED were not captured in those
%  screenshots and are reconstructed from the reported behaviour.

%% 1. Export the arm assembly from Onshape
% Pulls the CAD assembly down and writes a multibody description XML.
% Parts without assigned materials import with zero inertial properties -
% acceptable here because the solver is kinematic, not dynamic.

onshapeURL = "https://cad.onshape.com/documents/<documentId>/w/<workspaceId>/e/<elementId>";
multibodyDescriptionFile = smexportonshape(onshapeURL);

% Returns: 'Assembly11.xml'

%% 2. Import as a rigid body tree
smimport(multibodyDescriptionFile);          % builds the Simscape model
robot = importrobot("Assembly11");           % rigidBodyTree for the solver

% robot =
%   rigidBodyTree with properties:
%       NumBodies: 5
%       BodyNames: {'Body1' 'Body2' 'Body3' 'Body4' 'Body5'}
%        BaseName: 'Base'
%         Gravity: [0 -9.8066 0]
%      DataFormat: 'struct'

show(robot);                                  % visual check of the tree

%% 3. Home configuration
% Starting guess for the solver: the arm's rest pose.
q0 = homeConfiguration(robot);                % 1x5 struct: JointName, JointPosition

%% 4. Configure the generalized IK solver
gik = generalizedInverseKinematics;
gik.RigidBodyTree = robot;
gik.ConstraintInputs = {"position", "aiming"};

% SolverAlgorithm defaults to 'BFGSGradientProjection', which was left
% unchanged. Two constraints: where the gripper must be, and where it
% must point.

%% 5. Position constraint - where the gripper goes
% End effector is the gripper body. The report's console session used both
% the imported CAD name and the generic 'Body5' - they refer to the same
% body.

endEffector = "arm_gripper_bcio_v0_1__RIGID";   % or "Body5"

posTgt = constraintPositionTarget(endEffector);
posTgt.TargetPosition = [0.0 0.5 0.5];          % metres, base frame
posTgt.PositionTolerance = 0;
posTgt.Weights = 1;

%% 6. Aiming constraint - where the gripper points
aimCon = constraintAiming(endEffector);
aimCon.TargetPoint = [0.0 0.0 0.0];
aimCon.AngularTolerance = 0;
aimCon.Weights = 1;

%% 7. Solve
% INFERRED - the solve call itself is past the end of the captured session.
[qSol, solutionInfo] = gik(q0, posTgt, aimCon);

fprintf("Solver status : %s\n", solutionInfo.Status);
fprintf("Iterations    : %d\n", solutionInfo.Iterations);
fprintf("Pose error    : %.4f\n", solutionInfo.PoseErrorNorm);

show(robot, qSol);                              % visualise the solution

%% 8. Joint angles out
% INFERRED - the mapping from solver output to servo commands was not
% captured. Recorded behaviour: joint angles were converted to degrees and
% sent to the Arduino over serial, which drove the six arm servos.

jointAnglesRad = [qSol.JointPosition];
jointAnglesDeg = rad2deg(jointAnglesRad);

disp("Joint angles (deg):");
disp(jointAnglesDeg);

%% Notes
%  - The solver is kinematic only: no material properties were assigned in
%    CAD, so inertial terms are zero. Fine for pose solving, not usable for
%    torque or dynamics analysis.
%  - Reported accuracy: positional error under 5 mm in 90% of trials.
%  - Target coordinates came from the vision pipeline after a bottle or can
%    was detected and localised.
