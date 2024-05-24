

clc;
clear;
close all;

%% Create the search scenario

model = CreateModel(); % Create search map and parameters

CostFunction=@(x) MyCost(x,model);    % Cost Function

nVar = model.n;       % Number of Decision Variables = searching dimension of PSO = number of movements

VarSize=[nVar 2];   % Size of Decision Variables Matrix
VarMin= model.xmin+1;           % Lower Bound of particles (Variables)
VarMax = model.xmax -1;          % Upper Bound of particles 

%% PSO Parameters

MaxIt=200;          % Maximum Number of Iterations

nPop=500;           % Population Size (Swarm Size)

w=1;                % Inertia Weight
wdamp=0.98;         % Inertia Weight Damping Ratio
c1=2.5;             % Personal Learning Coefficient
c2=2.5;             % Global Learning Coefficient

alpha= 0.2;
VelMax=alpha*(VarMax-VarMin);    % Maximum Velocity
VelMin=-VelMax;                    % Minimum Velocity

%% Initialization

% Create an Empty Particle Structure
empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

% Initialize Global Best
GlobalBest.Cost = inf; % Maximization problem

% Create an empty particle matrix, each particle is a solution (searching path)
particle=repmat(empty_particle,nPop,1);
A = CreateRandomSolution(model);
% Initialization Loop
for i=1:nPop
    
    % Initialize Position
    particle(i).Position=CreateRandomSolution(model);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    costP = CostFunction(particle(i).Position);
    particle(i).Cost= costP;
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost < GlobalBest.Cost
        GlobalBest=particle(i).Best;
    end
    
end

% Array to Hold Best Cost Values at Each Iteration
BestCost=zeros(MaxIt,1);

%% PSO Main Loop
for it=1:MaxIt
    for i=1:nPop
                
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            + c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        %A = particle(i).Velocity;
        % Update Velocity Bounds
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Update Position Bounds
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        %B =  particle(i).Position
        
        % Evaluation
        costP = CostFunction(particle(i).Position);
        particle(i).Cost = costP;

        % Update Personal Best
        if particle(i).Cost < particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end
            
        end
      
    end
    
    % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;
    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
end

%% Results
path = GlobalBest.Position;
% Plot Solution
figure(1);
PlotSolution(path,model);  

% Plot Best Cost Over Iterations
figure(2);
plot(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
