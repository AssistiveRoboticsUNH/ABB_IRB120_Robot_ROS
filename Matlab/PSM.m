classdef PSM < handle
    properties (Constant)
        % Properties of object
        Name = 'PSM'
    end
    
    properties(SetAccess = public)
        % CLASS PUBLIC PROPERTIES
        % The following are parameters for each PSM segmentation
        a 
        b
        F
        t
        t0
        te
        x0
        xd0
        traj
        offset
    end
    
    
    methods

        function obj = PSM()
        % To use this implimetnatyion of PSM, you must first declare a PSM
        % object
        % psm = PSM()
        % Once a PSM object is intiated, you can use it to learn a
        % trajectory with the following.
        % psm.Learn_Traj(t,x,e)
        % where t and x are the time series trajectory data and e is the
        % acceptable error threshold as a percentage of the total trajecory length. 
        % Once a trajectory is learned, use psm.Generate_Traj(T,S) to
        % generate a new trajecotry 
        % T = time cosntant
        % S = scaling term
        % You can now use psm.traj{1}, psm.traj{2}, psm.traj{3} to access the postion, velocity, and
        % acceleration of the learned trajecory.

        end
        
        function Learn_Traj(obj,t,x,e)
            % This function learns the paramters for the PSM model with the error threshold e. 
            T = [0:1000]./1000;
            t = t/(max(t)-min(t)); % normalize time from [0 1]
            x = smooth(smooth(x,t(end)/10,'lowess'),t(end)/5,'lowess')'; % smoothness value here is arbitrary. This uses minimal basis fucntions.
            x = spline(t,x,T)+e/10*sin(T);
            obj.offset = x(1);
            x = x/(x(end)-x(1));
            x = x-min(x); % normalize x from [0 1]
            t=T;
            e = (max(x)-min(x))*e;
            % initialize variables
            dt = t(2)-t(1);
            xd= obj.d_dt(x,dt);
            xdd = obj.d_dt(xd,dt);
            knot=1;
            Fit = [];
            ind = [];
            xx=x;
            xxd =xd;
            xxdd=xdd;
            x0 = x(knot);
            xd0 = xd(knot);
            a=0;
            b=0;
            F=0;
            % learning process 
            for i = 2:1:length(x)
                a_old = a;
                b_old = b;
                F_old = F;
                t  =dt*([knot:i]-knot)';
                % find parameters of segment from knot to i 
                [a,b,F] = obj.LLR(x(knot:i),xd(knot:i),xdd(knot:i),xx(knot:i),xxd(knot:i),xxdd(knot:i));
                t  =dt*([knot:i+1]-knot)';
                xx(knot:i+1) = obj.sol(t,a,b,F,0,x0,xd0,1,1);
                xxd(knot:i+1) = obj.sold(t,a,b,F,0,x0,xd0,1,1);
                xxdd(knot:i+1) = obj.soldd(t,a,b,F,0,x0,xd0,1,1);
                xt = max(abs(x(knot:i)-xx(knot:i)));
                t  = dt*([knot:i]-knot);
                xx(knot:i) = obj.sol(t,a,b,F,0,x0,xd0,1,1);
                % if the error xt is grerater than e, create new segment.  
                if (abs(xt)>e && (i-knot)>2) 
                a = a_old;
                b = b_old;
                F = F_old;
                i=i-1;
                xx=x;
                xxd =xd;
                xxdd=xdd;
                Fit = [Fit;a,b,F];
                ind = [ind;knot,i];
                x0 = x(i);
                xd0 = xd(i);
                knot = i;
                end
            end

            if sum(size(ind))>2
            Fit = [Fit;a,b,F];
            ind = [ind;knot,i];
            elseif sum(size(ind))==0
            Fit = [Fit;a,b,F];
            ind = [ind;knot,i];
            end
            obj.x0 = x(ind(:,1));
            obj.xd0 = xd(ind(:,1));
            obj.t= [0:i-1]*dt;
            obj.t0 = obj.t((ind(:,1)))';
            obj.te = obj.t((ind(:,2)))';
            obj.a = Fit(:,1);
            obj.b = Fit(:,2);
            obj.F = Fit(:,3);
        end
        function [a,b,F] = LLR(obj,x,xd,xdd,xx,xxd,xxdd)
            % Least-squares linear regression: the function finds the plane
            % that fits the phase space curve with the least error while
            % considering a energy constraint.  
            C = trapz(x,xdd); % energy of actual trajectory
            f = trapz(x,ones(length(xx),1)); % energy from constant term ((integral of constant with respect to the trajecorties displacement))
            A = trapz(x,xxd); % energy from velocity term, (integral of the new approximated velocity xdd times the coefficient a with respect to the trajecorties displacement)
            B = trapz(x,xx); % energy from displacement term (integral of the new approximated displacement times the coefficient b with respect to the trajecorties displacement)
            A = [sum(xd.^2) , sum(xd.*x) , sum(xd) , A
             sum(xd.*x) , sum(x.^2) , sum(x) , B
             sum(xd) , sum(x) , length(x) , f
             A , B , f, 0];
            b = [sum(xdd.*xd); sum(xdd.*x); sum(xdd); C];
            q=A\b;
            a = q(1);
            b = q(2);
            F = q(3);
        end
      
        function X = Generate_Traj(obj,T,S)  
            % This function generates the learned trajecory scaled by T,
            % and S. The scaling term S should be set to the desired goal postion. The generated trjecory will start at psm.offset and end at S.
            S = (S-obj.offset); %
            ind = 1;
            for i = 1:length(obj.te) 
            ind = [ind ;find(obj.te(i)==obj.t,1)];
            end
        X = zeros(1,length(obj.t));
        Xd = zeros(1,length(obj.t));
        Xdd = zeros(1,length(obj.t));
        for i = 1:length(ind)-1 
        X(ind(i):ind(i+1)) = obj.sol(obj.t(ind(i):ind(i+1))*T,obj.a(i),obj.b(i),obj.F(i),obj.t0(i),obj.x0(i),obj.xd0(i),T,S);
        Xd(ind(i):ind(i+1)) = obj.sold(obj.t(ind(i):ind(i+1))*T,obj.a(i),obj.b(i),obj.F(i),obj.t0(i),obj.x0(i),obj.xd0(i),T,S);
        Xdd(ind(i):ind(i+1) ) =obj.soldd(obj.t(ind(i):ind(i+1))*T,obj.a(i),obj.b(i),obj.F(i),obj.t0(i),obj.x0(i),obj.xd0(i),T,S);
        end
        X(isnan(X))=0;
        Xd(isnan(Xd))=0;
        Xdd(isnan(Xdd))=0;
        X = (X)+obj.offset;
        obj.traj = {X,Xd,Xdd}; 
        end
        function X = sol(obj,t,a,b,F,t0,x0,xd0,T,S)
               % solution to phase space LTI equaution
            X = S*real(((x0+F./b-((xd0-(x0+F./b).*(a./2-sqrt(a.^2+4*b)./2)))./sqrt(a.^2+4*b)).*exp(1/2*(t/T-t0).*(a-sqrt(a.^2+4*b)))+(((xd0-(x0+F./b).*(a./2-sqrt(a.^2+4*b)./2)))./sqrt(a.^2+4*b)).*exp(1/2*(t/T-t0).*(a+sqrt(a.^2+4*b)))-F./b));
        end
        function Xd = sold(obj,t,a,b,F,t0,x0,xd0,T,S)
            % solution to phase space LTI equaution differtiated once
        Xd =S*real((((a-sqrt(a.^2+4*b))/(2*T)).*(x0+F./b-((xd0-(x0+F./b).*(a./2-sqrt(a.^2+4*b)./2)))./sqrt(a.^2+4*b)).*exp(1/2*(t/T-t0).*(a-sqrt(a.^2+4*b)))+((a+sqrt(a.^2+4*b))/(2*T)).*(((xd0-(x0+F./b).*(a./2-sqrt(a.^2+4*b)./2)))./sqrt(a.^2+4*b)).*exp(1/2*(t/T-t0).*(a+sqrt(a.^2+4*b)))));
        end
        function Xdd = soldd(obj,t,a,b,F,t0,x0,xd0,T,S)
            % solution to phase space LTI equaution differtiated twice
        Xdd=S.*real((((a-sqrt(a.^2+4*b))/(2*T)).^2.*(x0+F./b-((xd0-(x0+F./b).*(a./2-sqrt(a.^2+4*b)./2)))./sqrt(a.^2+4*b)).*exp(1/2*(t/T-t0).*(a-sqrt(a.^2+4*b)))+((a+sqrt(a.^2+4*b))/(2*T)).^2.*(((xd0-(x0+F./b).*(a./2-sqrt(a.^2+4*b)./2)))./sqrt(a.^2+4*b)).*exp(1/2*(t/T-t0).*(a+sqrt(a.^2+4*b)))));
        end
         function xd = d_dt(obj,x,dt)
            % take numerical derivative. To aviod the length of the array
            % changing, the function interpolates the value of the array to the second derivative. 
            s=size(x);
            if s(2)>s(1)
            x = [x x(end)+(x(end)-x(end-1))+((x(end)-x(end-1))-(x(end-1)-x(end-2)))];
            xd = diff(x)./dt;
            elseif s(1)>s(2)
            x = [x;x(end)+(x(end)-x(end-1))+((x(end)-x(end-1))-(x(end-1)-x(end-2)))];
            xd = diff(x)./dt;
            end
        end
    end

end