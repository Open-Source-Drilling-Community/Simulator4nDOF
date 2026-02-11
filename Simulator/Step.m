function [x,y] = step_4nDOF(p,x0,UU)
% function which solves the drillstring PDEs and ODEs for one time step of the outer loop (p.dt)

eps = 1E-8;

% Define convenience parameters
Pt = p.PL*p.NL;     % Total number of cells
dtTemp = p.dxM/max([p.c_t,p.c_a])*.4;  % As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability

dxl = 1/p.Pl;
dtl = dxl/p.omegaMAX;  % As per the CFL condition for the depth of cut PDE

n = max(ceil(p.dt/dtTemp),ceil(p.dt/dtl)); % number of iterations in the inner loop
dt = p.dt/n; % time step of inner loop

% Parse states - for distributed states each column is a section
f  = reshape(x0(1:Pt),[p.PL p.NL]);            % Pipe shear strain
o  = reshape(x0(Pt+1:2*Pt),[p.PL p.NL]);       % Pipe angular velocity
e  = reshape(x0(2*Pt+1:3*Pt),[p.PL p.NL]);     % Pipe axial strain
v  = reshape(x0(3*Pt+1:4*Pt),[p.PL p.NL]);     % Pipe axial velocity

OL  = [x0(4*Pt+1:4*Pt+p.NL)].'; % Lumped element angular velocity
VL  = [x0(4*Pt+p.NL+1:4*Pt+2*p.NL)].'; % Lumped element axial velocity
Otd = x0(4*Pt+2*p.NL+1);               % Top drive angular velocity
OS  = x0(4*Pt+2*p.NL+2:4*Pt+2*p.NL+1+p.NS).'; % Sleeve angular velocity, can be empty vector if no sleeves
l   = x0(4*Pt+2*p.NL+2+p.NS:4*Pt+2*p.NL+1+p.NS+p.Pl); % Depth of cut

Theta = [x0(4*Pt+2*p.NL+p.NS+p.Pl+2:4*Pt+3*p.NL+1+p.NS+p.Pl)].';  % Lumped element angular displacement
Theta_S = [x0(4*Pt+3*p.NL+p.NS+p.Pl+2:4*Pt+3*p.NL+1+2*p.NS+p.Pl)].';  % Sleeve angular displacement, can be empty vector if no sleeves
Xc  = [x0(4*Pt+3*p.NL+2*p.NS+p.Pl+2:4*Pt+4*p.NL+1+2*p.NS+p.Pl)].';   % Lumped element lateral displacement, x direction
Xc_dot  = [x0(4*Pt+4*p.NL+2*p.NS+p.Pl+2:4*Pt+5*p.NL+1+2*p.NS+p.Pl)].';   % Lumped element lateral velocity, x direction
Yc  = [x0(4*Pt+5*p.NL+2*p.NS+p.Pl+2:4*Pt+6*p.NL+1+2*p.NS+p.Pl)].';   % Lumped element lateral displacement, y direction
Yc_dot  = [x0(4*Pt+6*p.NL+2*p.NS+p.Pl+2:4*Pt+7*p.NL+1+2*p.NS+p.Pl)].';   % Lumped element lateral velocity, y direction
slip_condition = [x0(4*Pt+7*p.NL+2*p.NS+p.Pl+2:4*Pt+8*p.NL+1+2*p.NS+p.Pl)].'; % Slip condition evaluated at each lumped element
if p.useMudMotor
    Ostator = x0(4*Pt+8*p.NL+2+2*p.NS+p.Pl);
    Orotor = x0(4*Pt+8*p.NL+3+2*p.NS+p.Pl);
end

% Compute Riemann invariants for wave equations
TorsionalDownward = o + p.c_t.*f; % Downward traveling wave, torsional
TorsionalUpward = o - p.c_t.*f; % Upward traveling wave, torsional
AxialDownward = v + p.c_a.*e; % Downward traveling wave, axial
AxialUpward = v - p.c_a.*e; % Upward traveling wave, axial

% Get dynamic axial drag from wave equations, and interpolate it at lumped element locations
drag = reshape(e,[p.PL p.NL]).*(ones(p.PL,1).*p.E.*p.A);
drag = interp1(p.x,drag(:),p.xL,'linear','extrap');

% Recompute string tension and normal forces based on dynamic axial drag
tension = fliplr(cumtrapz(p.xL,fliplr(p.dsigma_dx))) + p.axialBuoyancyForceChangeOfDiameters - drag;  % [N] Tension profile

fN = sqrt(((tension  + p.normalBuoyancyForceChangeOfDiameters).*[0,p.thetaVec_dot] - p.Wb.*sin([0,p.thetaVec])).^2 + ((tension + p.normalBuoyancyForceChangeOfDiameters).*[0,p.phiVec_dot].*sin([0,p.thetaVec])).^2); % [N/m] Normal force per length
I_fN = cumtrapz(p.xL,fN);

F_N_softstring = diff(I_fN); % [N] Lumped normal force per element assuming soft-string model (not used in 4nDOF model)

% compressive force including the bending moments due to fluid differential pressure
F_comp = [p.Ai(1),p.Ai].*(p.stringPressure - p.hydrostaticStringPressure)*(1-2*p.nu) - [p.Ao(1),p.Ao].*(p.annularPressure - p.hydrostaticAnnularPressure)*(1-2*p.nu) - tension;

tension = -F_comp;

% update bending stiffness with the new compressive force
kb = [p.kb(1) p.kb] - pi^2*F_comp./(2*p.Lb);
kb = max(kb,0); % kb < 0 would correspond to buckling which is not handled in our model

% compute pre-stressed normal forces
torque = reshape(f,[p.PL p.NL]).*(ones(p.PL,1).*p.J.*p.G);
torque = interp1(p.x,torque(:),p.xL,'linear','extrap');

% normal force components in Frenet-Serret coordinate system
fB = p.Wb.*[p.bz(1),p.bz] + [p.curvature(1),p.curvature].*[0,diff(torque)/p.dxL] + [p.curvature_dot(1),p.curvature_dot].*torque  - 2*[p.E(1),p.E].*[p.I(1),p.I].*[p.curvature_dot(1),p.curvature_dot].*[p.torsion(1),p.torsion] - [p.E(1),p.E].*[p.I(1),p.I].*[p.curvature(1),p.curvature].*[p.torsion_dot(1),p.torsion_dot];
fN = [p.curvature(1),p.curvature].*(tension + p.normalBuoyancyForceChangeOfDiameters) + p.Wb.*[p.nz(1),p.nz] - [p.E(1),p.E].*[p.I(1),p.I].*[p.curvature_ddot(1),p.curvature_ddot] + [p.E(1),p.E].*[p.I(1),p.I].*[p.curvature(1),p.curvature].*[p.torsion(1),p.torsion].^2 + - [p.curvature(1),p.curvature].*[p.torsion(1),p.torsion].*torque;
I_fB = cumtrapz(p.xL,fB);
I_fN = cumtrapz(p.xL,fN);
F_B_prestress = diff(I_fB); % pre-stressed force in binormal direction
F_N_prestress = diff(I_fN); % pre-stressed force in normal direction
F_N_prestress = F_N_prestress + UU.F_N_ext; % add external normal forces

% toolface angle used to convert normal force components from Frenet Serret frame to the cartesian coordinate frame (x-y)
sign_tf = sign((p.hy.*p.nz - p.hz.*p.ny).*p.tx + (p.hz.*p.nx - p.hx.*p.nz).*p.ty + (p.hx.*p.ny - p.hy.*p.nx).*p.tz); % sign of the toolface angle relative to the high side
tf_angle = real(acos(p.hx.*p.nx+p.hy.*p.ny+p.hz.*p.nz).*sign_tf); % tool-face angle
    
%% Solve lumped and distributed equations

for i=1:n
    
    Vtd = UU.v0; % Top drive axial velocity
    
    %% Interfaces
    OL_vec = [Otd OL];
    VL_vec = [Vtd VL];
    
    % Left boundaries
    TorsionalDownward_left = -TorsionalUpward(1,:) + 2*OL_vec(1:end-1);
    AxialDownward_left = -AxialUpward(1,:) + 2*VL_vec(1:end-1);
    
    % Right boundaries
    TorsionalUpward_right = -TorsionalDownward(p.PL,:) + 2*OL_vec(2:end);
    AxialUpward_right = -AxialDownward(p.PL,:) + 2*VL_vec(2:end);
    
    %% Bit rock interaction
    % Bit velocity
    vb = 1/2*(AxialDownward(p.PL,end)+AxialUpward(p.PL,end));
    
    if ~p.useMudMotor
        ob = 1/2*(TorsionalDownward(p.PL,end)+TorsionalUpward(p.PL,end));
    else
        ob = Orotor;
    end
    
    if p.BRIModel == 1 
    %% Detournay model
        % Combined depth of cut
        d = p.N * max( l(end) , 0);

        epsilon = 2*pi*0.2;  % regularization term to avoid numerical issues at zero bit velocity
        reg = ob/sqrt(ob^2 + epsilon^2);

        wc = d*p.Rb*p.zeta*p.epsilon;   % Cutting component of weight on bit
        tc = d*p.Rb^2*p.epsilon/2*reg^2; % Cutting component of bit torque

        % g function nonlinearity for friction components
        % ga = 1/p.Wf * ( AxialDownward(p.PL,end)*p.A(end)*p.E(end)/p.c_a - wc);
        % ga = (vb>0.001) + ((vb)<=0.001) * min( max(ga,0), 1);
        % Comment out to prevent instability when tagging bottom 
        ga = 1;

        wf = p.Wf*ga; % Friction component of weight on bit
        tf = 0.5*(1+exp(-p.bitRockFrictionExponent * ob / (2.0 * pi)))*p.Tf*ga; % Friction component of torque on bit

        % Additional terms for force and torque at bit during stick situation
        g_tt = TorsionalDownward(p.PL,end)*p.J(end)*p.G(end)/p.c_t - tc - tf;
        g_tt = (ob<0.1) * min(g_tt,0);
        g_wt = AxialDownward(p.PL,end)*p.A(end)*p.E(end)/p.c_a - wc - wf;
        g_wt = g_wt * (abs(g_tt)>1e-3);

        if p.onBottom
            tb = tc + tf + g_tt; % Torque on bit
            wb = wc + wf + g_wt; % Weight on bit
            l_pad = [0; l]; % add left padding for solving depth of cut PDE
            l = l - dt/dxl *max(ob,0)*p.N/(2*pi) *diff(l_pad) + dt*vb;
        else
            tb = 0;
            wb = 0;
        end
    else
        %% MSE model
        if p.onBottom
            l(end) = (1 - p.alpha_ROP)*l(end) + p.alpha_ROP*2*pi*vb/ob*(ob>0.5); % Low-pass filtered depth of cut
            l(end) = max(l(end),0);
            mu_b = p.mu*0.5*(1+exp(-p.bitRockFrictionExponent * ob / (2.0 * pi)));
            wb = max(pi*p.Rb^2*p.CCS/p.bitEfficiencyFactor/(1+2*mu_b*p.Rb/(3*l(end))),0); % Weight on bit
            tb = 2/3*mu_b*p.Rb*wb; % Torque on bit
        else
            tb = 0;
            wb = 0;
        end
    end
    
    %% Augment Riemann invariants with interface values
    a_pad = [TorsionalDownward_left; TorsionalDownward];
    b_pad = [TorsionalUpward; TorsionalUpward_right];
    u_pad = [AxialDownward_left; AxialDownward];
    v_pad = [AxialUpward; AxialUpward_right];
    
    %% 1st order Upwind (Note the staggered procedure for improved stability - first update the distributed states, then the lumped states)
    TorsionalDownward = TorsionalDownward - dt/p.dxM *p.c_t .*diff(a_pad);
    TorsionalUpward = TorsionalUpward + dt/p.dxM *p.c_t .*diff(b_pad);
    
    AxialDownward = AxialDownward - dt/p.dxM *p.c_a .*diff(u_pad);    
    AxialUpward = AxialUpward + dt/p.dxM *p.c_a .*diff(v_pad);
    
    
    %% Compute torques ('tau') and forces ('for') for the lumped elements
    tauTD = p.J(1)*p.G(1)/(2*p.c_t) *(TorsionalDownward(1,1)-TorsionalUpward(1,1)); % torque from the top drive on the first distributed element
    % tau_m - acting on the top of the element, tau_p acting on the bottom
    tau_m = p.J.*p.G              ./(2*p.c_t).*(TorsionalDownward(p.PL,:) -TorsionalUpward(p.PL,:)); % every other lumped element apart from top drive
    temp  = p.J(2:end).*p.G(2:end)./(2*p.c_t).*(TorsionalDownward(1,2:end)-TorsionalUpward(1,2:end));
    
    
    if ~p.useMudMotor
        tm = 0;
        tau_p = [temp tb]; % add bit boundary condition to last lumped element
    else
        if (Orotor-Ostator) / p.omega0_motor <= 1
            tm = p.T_max_motor*(1 - (Orotor-Ostator) / p.omega0_motor).^(1/p.alpha_motor);
        else
            tm = -p.P0_motor*p.V_motor*((Orotor-Ostator) / p.omega0_motor - 1);
        end
        
        tau_p = [temp tm]; % add mud motor boundary condition to last lumped element
    end
    
    % for_m - acting on the top of the element, for_p acting on the bottom
    for_m = p.A.*p.E              ./(2*p.c_a) .*(AxialDownward(p.PL,:) -AxialUpward(p.PL,:));
    temp  = p.A(2:end).*p.E(2:end)./(2*p.c_a) .*(AxialDownward(1,2:end)-AxialUpward(1,2:end));
    for_p = [temp wb];  % add bit boundary condition to last lumped element
    
    % Compute normal forces based on linear elastic contact model
    rc = sqrt(Xc.^2 + Yc.^2);
    phi = atan2(Yc,Xc); % whirl angle (rad)
    r_dot = Xc_dot.*cos(phi) + Yc_dot.*sin(phi);
    iC = rc >= p.rc;
    F_N = p.kw*(rc - p.rc).*iC + p.dw*r_dot.*iC;
    F_N(end-1) = F_N(end-1) + UU.Fshock; % add a lateral shock at the element above the bit
    phi_dot = 1./(rc.^2+eps).*(Yc_dot.*Xc - Xc_dot.*Yc); % whirl velocity (rad/s)

    Xc_iMinus1 = [0,Xc(1:end-1)];
    Xc_iPlus1 = [Xc(2:end),0];
    Yc_iMinus1 = [0,Yc(1:end-1)];
    Yc_iPlus1 = [Yc(2:end),0];
    
    % lateral forces due to bending
    Fk_x = -(kb(1:end-1)+kb(2:end)).*Xc + kb(1:end-1).*Xc_iMinus1 + kb(2:end).*Xc_iPlus1;
    Fk_y = -(kb(1:end-1)+kb(2:end)).*Yc + kb(1:end-1).*Yc_iMinus1 + kb(2:end).*Yc_iPlus1; 
    
    % lateral forces due to pre-stressed configuration
    F_prestress_x = F_N_prestress.*sin(tf_angle) + F_B_prestress.*cos(tf_angle);
    F_prestress_y = F_N_prestress.*cos(tf_angle) - F_B_prestress.*sin(tf_angle);
       
    % lateral forces due to fluid-structure interaction
    Ff_x = -p.Df*Xc_dot - p.Mf.*OL.*Yc_dot - p.Df*OL.*Yc/2 + p.Mf.*OL.^2.*Xc/4;
    Ff_y = -p.Df*Yc_dot + p.Mf.*OL.*Xc_dot + p.Df*OL.*Xc/2 + p.Mf.*OL.^2.*Yc/4;
    Ff_x(p.iS) = -p.Df*Xc_dot(p.iS) - p.Mf(p.iS).*OS.*Yc_dot(p.iS) - p.Df*OS.*Yc(p.iS)/2 + p.Mf(p.iS).*OS.^2.*Xc(p.iS)/4;
    Ff_y(p.iS) = -p.Df*Yc_dot(p.iS) + p.Mf(p.iS).*OS.*Xc_dot(p.iS) + p.Df*OS.*Xc(p.iS)/2 + p.Mf(p.iS).*OS.^2.*Yc(p.iS)/4;
    
    % sleeve braking force
    F_S = p.k_ec.*(OL(p.iS) - OS); % torsional damping
    
    %% Coulomb friction
    %
    V_a = VL;
    V_t = phi_dot.*rc+ OL.*p.ro;
    V_t(p.iS) = phi_dot(p.iS).*rc(p.iS) + OS.*p.r_So; % replace with the sleeve angular velocity if there is a sleeve
    
    % Compute forces required to enter static friction condition (i.e., no axial movement and rolling without slip)
    Fstatic_a = VL.*p.M_L/dt + for_m-for_p-p.ka*VL;
    
    theta_ddot_noslip = 1./(p.I_L+(p.M_L + p.Mf).*p.ro.^2 - p.M_e.*p.e.*p.ro.*cos(Theta - phi)).*((p.M_L + p.Mf).*p.ro.*r_dot.*phi_dot + p.ro.*(Fk_x + F_prestress_x + Ff_x).*sin(phi) - p.ro.*(Fk_y + F_prestress_y + Ff_y).*cos(phi) + tau_m-tau_p-p.kt*OL - p.M_e.*p.e.*p.ro.*OL.^2.*sin(Theta - phi));
    theta_ddot_noslip(p.iS) = 1./(p.I_S+(p.M_L(p.iS) + p.Mf(p.iS)).*p.r_So.^2 - p.M_e(p.iS).*p.e(p.iS).*p.r_So.*cos(Theta_S - phi(p.iS))).*((p.M_L(p.iS) + p.Mf(p.iS)).*p.r_So.*r_dot(p.iS).*phi_dot(p.iS) + p.r_So.*(Fk_x(p.iS) + F_prestress_x(p.iS) + Ff_x(p.iS)).*sin(phi(p.iS)) - p.r_So.*(Fk_y(p.iS) + F_prestress_y(p.iS) + Ff_y(p.iS)).*cos(phi(p.iS)) + p.r_Si*F_S - p.M_e(p.iS).*p.e(p.iS).*p.r_So.*OS.^2.*sin(Theta_S - phi(p.iS)));
    phi_ddot_noslip =  1./(p.M_L + p.Mf + p.I_L./p.ro.^2 - p.M_e.*p.e./p.ro.*cos(Theta - phi)).*((-2*(p.M_L + p.Mf) - p.I_L./p.ro.^2 + p.M_e.*p.e./p.ro.*cos(Theta - phi)).*r_dot.*phi_dot - (Fk_x + F_prestress_x + Ff_x).*sin(phi) + (Fk_y + F_prestress_y + Ff_y).*cos(phi) - 1./p.ro.*(tau_m-tau_p-p.kt*OL) + p.M_e.*p.e.*OL.^2.*sin(Theta - phi));
    phi_ddot_noslip(p.iS) =  1./(p.M_L(p.iS) + p.Mf(p.iS) + p.I_S./p.r_So.^2 - p.M_e(p.iS).*p.e(p.iS)./p.r_So.*cos(Theta_S - phi(p.iS))).*((-2*(p.M_L(p.iS) + p.Mf(p.iS)) - p.I_S./p.r_So.^2 + p.M_e(p.iS).*p.e(p.iS)./p.r_So.*cos(Theta_S - phi(p.iS))).*r_dot(p.iS).*phi_dot(p.iS) - (Fk_x(p.iS) + F_prestress_x(p.iS) + Ff_x(p.iS)).*sin(phi(p.iS)) + (Fk_y(p.iS) + F_prestress_y(p.iS) + Ff_y(p.iS)).*cos(phi(p.iS)) - p.r_Si./p.r_So.*F_S + p.M_e(p.iS).*p.e(p.iS).*OS.^2.*sin(Theta_S - phi(p.iS)));
    Fstatic_t = (p.I_L./(p.ro.^2)).*(r_dot.*phi_dot + rc.*phi_ddot_noslip) + 1./p.ro.*(tau_m-tau_p-p.kt*OL);
    Fstatic_t(p.iS) = (p.I_S./(p.r_So.^2)).*(r_dot(p.iS).*phi_dot(p.iS) + rc(p.iS).*phi_ddot_noslip(p.iS)) + p.r_Si./p.r_So.*F_S;
    
    % Coulomb force
    VBar    = sqrt( V_a.^2 + V_t.^2 ) +eps; % eps is important for numerics
    FResBar = sqrt( Fstatic_a.^2 + Fstatic_t.^2 ) + eps;
    
    FcTh = F_N.*p.mu_s; % static force
    
    % evaluate slip condition; if the element was previously rolling without slip, we check if the resultant force exceeds the static threshold;
    % otherwise, we check if the total velocity is close to zero to go back into a rolling condition
    for j=1:p.NL
        if ~slip_condition(j)
            slip_condition(j) = (FResBar(j) > FcTh(j));
        else
            if VBar(j) < p.v_c
                slip_condition(j) = 0;
            end
        end
    end
    
    Fc =  (~slip_condition).* max(min(FResBar,FcTh),-FcTh) + slip_condition.* F_N.*(p.mu_k + (p.mu_s - p.mu_k).*exp(-p.stribeck*VBar));
    Fc_a = (~slip_condition).*Fc.*(Fstatic_a./FResBar) + slip_condition.*Fc.*V_a./VBar;
    Fc_t = (~slip_condition).*Fc.*(Fstatic_t./FResBar) + slip_condition.*Fc.*V_t./VBar;
    
    % for the elements with sleeves, account for reduction in axial friction due to the spur wheels
    Fc_a(p.iS) = Fc_a(p.iS)*(1-p.axialFrictionReduction);
    Fc_t(p.iS) = sqrt(Fc(p.iS).^2 - Fc_a(p.iS).^2).*sign(Fc_t(p.iS));
    
    %% Solve Lumped ODEs - check for numerical instability (fix by decreasing dt, increasing I_L, M_L, or increasing p.kt, p.ka, p.kl)
    Otd = Otd + 1/p.I_TD * dt*(UU.tau_Motor-tauTD);
    
    OL_dot = 1./p.I_L .*(tau_m-tau_p-p.kt*OL-Fc_t.*p.ro);
    % replace tangential friction force with sleeve braking force on the pipe elements with sleeves 
    OL_dot(p.iS) = 1./p.I_L(p.iS) .*(tau_m(p.iS)-tau_p(p.iS)-p.kt*OL(p.iS)-F_S.*p.ro(p.iS));
    OS_dot = 1./p.I_S .* dt.*(F_S.*p.r_Si-Fc_t(p.iS).*p.r_So);
    VL_dot = 1./p.M_L .*(for_m-for_p-p.ka*VL-Fc_a);
    
    Theta = Theta + OL*dt;
    Theta_S = Theta_S + OS*dt;
    
    OL = OL + OL_dot*dt;
    OS = OS + OS_dot*dt;
    VL = VL + VL_dot*dt;
    
    % lateral forces due to mass imbalance
    Fe_x = p.M_e.*p.e.*(OL.^2.*cos(Theta) + OL_dot.*sin(Theta));
    Fe_y = p.M_e.*p.e.*(OL.^2.*sin(Theta) - OL_dot.*cos(Theta));
    Fe_x(p.iS) = p.M_e(p.iS).*p.e(p.iS).*(OS.^2.*cos(Theta_S) + OS_dot.*sin(Theta_S));
    Fe_y(p.iS) = p.M_e(p.iS).*p.e(p.iS).*(OS.^2.*sin(Theta_S) - OS_dot.*cos(Theta_S));
    
    % lateral accelerations in cartesian coordinates
    Xc_ddot = 1./(p.M_L + p.Mf).*(Fk_x + F_prestress_x + Ff_x + Fe_x - p.kl*Xc_dot - F_N.*cos(phi) + Fc_t.*sin(phi));
    Yc_ddot = 1./(p.M_L + p.Mf).*(Fk_y + F_prestress_y + Ff_y + Fe_y - p.kl*Yc_dot - F_N.*sin(phi) - Fc_t.*cos(phi));
    
    Xc = Xc + Xc_dot*dt;
    Xc_dot = Xc_dot + Xc_ddot*dt;
    Yc = Yc + Yc_dot*dt;
    Yc_dot = Yc_dot + Yc_ddot*dt;
    
    % Mud motor
    if p.useMudMotor
        Ostator = OL(end);
        Orotor = Orotor + dt/(p.I_rotor+p.M_rotor*p.delta_rotor^2*p.N_rotor^2)*(OL_dot(end)*p.M_rotor*p.delta_rotor^2*p.N_stator*p.N_rotor + tm - tb);
    end
end

%%
% Compute states from Riemann invariants
o = 1/2        *(TorsionalDownward+TorsionalUpward);
f = 1/(2*p.c_t)*(TorsionalDownward-TorsionalUpward);

v = 1/2        *(AxialDownward+AxialUpward);
e = 1/(2*p.c_a)*(AxialDownward-AxialUpward);

% Parse states
x(1:Pt)               = f(:);   % Pipe shear strain
x(Pt+1:2*Pt)          = o(:);   % Pipe angular velocity
x(2*Pt+1:3*Pt)        = e(:);   % Pipe axial strain
x(3*Pt+1:4*Pt)        = v(:);   % Pipe axial velocity

x(4*Pt+1:4*Pt+p.NL)        = OL.'; % Lumped element angular velocity
x(4*Pt+p.NL+1:4*Pt+2*p.NL) = VL.'; % Lumped element axial velocity
x(4*Pt+2*p.NL+1)           = Otd;  % Top-drive angular velocity
x(4*Pt+2*p.NL+2:4*Pt+2*p.NL+1+p.NS) = OS.'; % Sleeve angular velocity
x(4*Pt+2*p.NL+2+p.NS:4*Pt+2*p.NL+1+p.NS+p.Pl) = l';
x(4*Pt+2*p.NL+p.NS+p.Pl+2:4*Pt+3*p.NL+1+p.NS+p.Pl) = Theta; % Lumped element angular displacement
x(4*Pt+3*p.NL+p.NS+p.Pl+2:4*Pt+3*p.NL+1+2*p.NS+p.Pl) = Theta_S; % Sleeve angular displacement
x(4*Pt+3*p.NL+2*p.NS+p.Pl+2:4*Pt+4*p.NL+1+2*p.NS+p.Pl) = Xc.'; % Lumped element lateral displacement, x direction
x(4*Pt+4*p.NL+2*p.NS+p.Pl+2:4*Pt+5*p.NL+1+2*p.NS+p.Pl) = Xc_dot.'; % Lumped element lateral velocity, x direction
x(4*Pt+5*p.NL+2*p.NS+p.Pl+2:4*Pt+6*p.NL+1+2*p.NS+p.Pl) = Yc.'; % Lumped element lateral displacement, y direction
x(4*Pt+6*p.NL+2*p.NS+p.Pl+2:4*Pt+7*p.NL+1+2*p.NS+p.Pl) = Yc_dot.'; % Lumped element lateral velocity, y direction
x(4*Pt+7*p.NL+2*p.NS+p.Pl+2:4*Pt+8*p.NL+1+2*p.NS+p.Pl) = slip_condition;
if p.useMudMotor
    x(4*Pt+8*p.NL+2+2*p.NS+p.Pl) = Ostator; % Mud motor stator angular velocity
    x(4*Pt+8*p.NL+3+2*p.NS+p.Pl) = Orotor; % Mud motor rotor angular velocity
end

% Bending moments
Mb_x = p.E.*p.I.*(Xc_iPlus1 - 2*Xc + Xc_iMinus1) ./ p.dxL.^2; % Bending moment x-component
Mb_y = p.E.*p.I.*(Yc_iPlus1 - 2*Yc + Yc_iMinus1) ./ p.dxL.^2; % Bending moment y-component
    
if p.usePipeMovementReconstruction % compute additional variables used for pipe movement reconstruction

    r_ddot = Xc_ddot(p.idx_sensor)*cos(phi(p.idx_sensor)) + Yc_ddot(p.idx_sensor)*sin(phi(p.idx_sensor)) - Xc_dot(p.idx_sensor)*phi_dot(p.idx_sensor)*sin(phi(p.idx_sensor)) + Yc_dot(p.idx_sensor)*phi_dot(p.idx_sensor)*cos(phi(p.idx_sensor));
    phi_ddot = slip_condition(p.idx_sensor)*(1/(rc(p.idx_sensor)^2+eps)*(Yc_ddot(p.idx_sensor)*Xc(p.idx_sensor) - Xc_ddot(p.idx_sensor)*Yc(p.idx_sensor)) - 2*r_dot(p.idx_sensor)*phi_dot(p.idx_sensor)/(rc(p.idx_sensor)+eps)) + (~slip_condition(p.idx_sensor))*phi_ddot_noslip(p.idx_sensor);
    phi0_sensor = 0; 
    
    if ~ismember(p.idx_sensor,p.iS)
        r0_sensor = 0.5*(p.ri(p.idx_sensor)+p.ro(p.idx_sensor)); % radial position of accelerometer relative to pipe centerline
        theta_ddot =  slip_condition(p.idx_sensor).*OL_dot(p.idx_sensor) + (~slip_condition(p.idx_sensor)).*theta_ddot_noslip(p.idx_sensor);
        u_x = Xc(p.idx_sensor) + r0_sensor*cos(Theta(p.idx_sensor)) - phi0_sensor*sin(Theta(p.idx_sensor));
        u_y = Yc(p.idx_sensor) + phi0_sensor*cos(Theta(p.idx_sensor)) + r0_sensor*sin(Theta(p.idx_sensor));
        u_x_ddot = Xc_ddot(p.idx_sensor) + r0_sensor*(-OL(p.idx_sensor)^2*cos(Theta(p.idx_sensor)) - OL_dot(p.idx_sensor)*sin(Theta(p.idx_sensor))) + phi0_sensor*(OL(p.idx_sensor)^2*sin(Theta(p.idx_sensor)) - OL_dot(p.idx_sensor)*cos(Theta(p.idx_sensor)));
        u_y_ddot = Yc_ddot(p.idx_sensor) + phi0_sensor*(-OL(p.idx_sensor)^2*cos(Theta(p.idx_sensor)) - OL_dot(p.idx_sensor)*sin(Theta(p.idx_sensor))) - r0_sensor*(OL(p.idx_sensor)^2*sin(Theta(p.idx_sensor)) - OL_dot(p.idx_sensor)*cos(Theta(p.idx_sensor)));
    else
        idx_sleeve_sensor = p.iS == p.idx_sensor;
        r0_sensor = 0.5*(p.r_Si + p.r_So);      
        theta_ddot =  slip_condition(p.idx_sensor)*OS_dot(idx_sleeve_sensor) + (~slip_condition(p.idx_sensor))*theta_ddot_noslip(p.idx_sensor);
        u_x = Xc(p.idx_sensor) + r0_sensor*cos(Theta_S(idx_sleeve_sensor)) - phi0_sensor*sin(Theta_S(idx_sleeve_sensor));
        u_y = Yc(p.idx_sensor) + phi0_sensor*cos(Theta_S(idx_sleeve_sensor)) + r0_sensor*sin(Theta_S(idx_sleeve_sensor));
        u_x_ddot = Xc_ddot(p.idx_sensor) + r0_sensor*(-OS(idx_sleeve_sensor)^2*cos(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*sin(Theta_S(idx_sleeve_sensor))) + phi0_sensor*(OS(idx_sleeve_sensor)^2*sin(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*cos(Theta_S(idx_sleeve_sensor)));
        u_y_ddot = Yc_ddot(p.idx_sensor) + phi0_sensor*(-OS(idx_sleeve_sensor)^2*cos(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*sin(Theta_S(idx_sleeve_sensor))) - r0_sensor.*(OS(idx_sleeve_sensor)^2*sin(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*cos(Theta_S(idx_sleeve_sensor)));
    end
    
    if ~ismember(p.idx_sensor-1,p.iS)
        r0_sensor = 0.5*(p.ri(p.idx_sensor-1)+p.ro(p.idx_sensor-1));
        u_x_iMinus1 = Xc(p.idx_sensor-1) + r0_sensor*cos(Theta(p.idx_sensor-1)) - phi0_sensor*sin(Theta(p.idx_sensor-1));
        u_y_iMinus1 = Yc(p.idx_sensor-1) + phi0_sensor*cos(Theta(p.idx_sensor-1)) + r0_sensor*sin(Theta(p.idx_sensor-1));
        u_x_ddot_iMinus1 = Xc_ddot(p.idx_sensor-1) + r0_sensor*(-OL(p.idx_sensor-1)^2*cos(Theta(p.idx_sensor-1)) - OL_dot(p.idx_sensor-1)*sin(Theta(p.idx_sensor-1))) + phi0_sensor*(OL(p.idx_sensor-1)^2*sin(Theta(p.idx_sensor-1)) - OL_dot(p.idx_sensor-1)*cos(Theta(p.idx_sensor-1)));
        u_y_ddot_iMinus1 = Yc_ddot(p.idx_sensor-1) + phi0_sensor*(-OL(p.idx_sensor-1)^2*cos(Theta(p.idx_sensor-1)) - OL_dot(p.idx_sensor-1)*sin(Theta(p.idx_sensor-1))) - r0_sensor*(OL(p.idx_sensor-1)^2*sin(Theta(p.idx_sensor-1)) - OL_dot(p.idx_sensor-1)*cos(Theta(p.idx_sensor-1)));
    else 
        idx_sleeve_sensor = p.iS == p.idx_sensor-1;
        u_x_iMinus1 = Xc(p.idx_sensor-1) + r0_sensor*cos(Theta_S(idx_sleeve_sensor)) - phi0_sensor*sin(Theta_S(idx_sleeve_sensor));
        u_y_iMinus1 = Yc(p.idx_sensor-1) + phi0_sensor*cos(Theta_S(idx_sleeve_sensor)) + r0_sensor*sin(Theta_S(idx_sleeve_sensor));
        u_x_ddot_iMinus1 = Xc_ddot(p.idx_sensor-1) + r0_sensor*(-OS(idx_sleeve_sensor)^2*cos(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*sin(Theta_S(idx_sleeve_sensor))) + phi0_sensor*(OS(idx_sleeve_sensor)^2*sin(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*cos(Theta_S(idx_sleeve_sensor)));
        u_y_ddot_iMinus1 = Yc_ddot(p.idx_sensor-1) + phi0_sensor*(-OS(idx_sleeve_sensor)^2*cos(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*sin(Theta_S(idx_sleeve_sensor))) - r0_sensor.*(OS(idx_sleeve_sensor)^2*sin(Theta_S(idx_sleeve_sensor)) - OS_dot(idx_sleeve_sensor)*cos(Theta_S(idx_sleeve_sensor)));
    end
  
    % Bending angles
    Theta_x = -(u_y - u_y_iMinus1)/ p.dxL; % Bending angle x-component
    Theta_y = (u_x - u_x_iMinus1)/ p.dxL; % Bending angle y-component
    Theta_x_ddot = -(u_y_ddot - u_y_ddot_iMinus1)/ p.dxL; % Bending angle second derivative x-component
    Theta_y_ddot = (u_x_ddot - u_x_ddot_iMinus1)/ p.dxL; % Bending angle second derivative y-component
end

% Parse outputs
y(1:p.NL) = F_N; % Normal force profile
y(p.NL+1:2*p.NL+1) = tension; % Tension profile
y(2*p.NL+2:3*p.NL+1) = Mb_x; % Bending moment y-component profile
y(3*p.NL+2:4*p.NL+1) = Mb_y; % Bending moment z-component profile
y(4*p.NL+2:5*p.NL+1) = Fc_t; % Tangential force profile

y(5*p.NL+2) = wb;  % Weight on bit
y(5*p.NL+3) = tb;  % Torque on bit

% outputs for pipe movement reconstruction
if p.usePipeMovementReconstruction
    y(5*p.NL+4) = VL(p.idx_sensor); % axial velocity
    if ismember(p.idx_sensor,p.iS)
        idx_sleeve_sensor = p.iS == p.idx_sensor;
        y(5*p.NL+5) = Theta_S(idx_sleeve_sensor); % sleeve angular displacement
        y(5*p.NL+6) = OS(idx_sleeve_sensor); % sleeve angular velocity
    else
        y(5*p.NL+5) = Theta(p.idx_sensor); % pipe angular displacement
        y(5*p.NL+6) = OL(p.idx_sensor); % pipe angular velocity
    end
    y(5*p.NL+7) = rc(p.idx_sensor); % radial position
    y(5*p.NL+8) = phi(p.idx_sensor); % whirl angle
    y(5*p.NL+9) = r_dot(p.idx_sensor); % radial velocity
    y(5*p.NL+10) = phi_dot(p.idx_sensor); % whirl velocity
    y(5*p.NL+11) = VL_dot(p.idx_sensor); % axial acceleration
    y(5*p.NL+12) = theta_ddot; % angular acceleration
    y(5*p.NL+13) = r_ddot; % radial acceleration
    y(5*p.NL+14) = phi_ddot; % whirl acceleration
    y(5*p.NL+15) = Theta_x; % bending angle x-component
    y(5*p.NL+16) = Theta_y; % bending angle y-component
    y(5*p.NL+17) = Theta_x_ddot; % bending angle second derivative x-component
    y(5*p.NL+18) = Theta_y_ddot; % bending angle second derivative y-component
end