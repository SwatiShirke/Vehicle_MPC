function  fx=fricx(Slips ,F_z)
persistent B C D E;
if isempty(B)
B=11.275; C=1.56; D=1; E=-0.97;
end

%alpha=-Slips;
%aB=asin(sin(alpha));
f = D  * sin(C * atan(B * Slips - E * (B * Slips - atan(B * Slips))));
fx=f.*F_z;


end

% for i=1:4
%     if fx(i)*Slips(i)>0
%         fx(i)=-fx(i);
%     end
% end
% for i=1:2
%     if fx(i)>0
%         fx(i)=0;
%     end
% end


% function fx=fricx(lambda,Fz)
% persistent B C D E;
% if isempty(B)
% B=11.275; C=1.56; D=-.95; E=-1.999;
% end
% alpha=-lambda;
% aB=asin(sin(alpha));
% f=D*sin(C*atan(B*(1-E)*aB+E*atan(B*aB)));
% fx=f.*Fz;
% for i=1:4
%     if fx(i)*alpha(i)>0
%         fx(i)=-fx(i);
%     end
% end
% for i=1:2
%     if fx(i)>0
%         fx(i)=0;
%     end
% end