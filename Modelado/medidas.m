dif = abs(length(out.time) - length(in.time));
out = out(dif + 1:end,:);
%Eje X
VinX = [in.fieldtwistlinearx in.time];
VoutX = [out.fieldtwistlinearx out.time];
%Eje Y
VinY = [in.fieldtwistlineary in.time];
VoutY = [out.fieldtwistlineary out.time];
%Eje Z
VinZ = [in.fieldtwistlinearz in.time];
VoutZ = [out.fieldtwistlinearz out.time];
%Giro
VinYaw = [in.fieldtwistangularz in.time];
VoutYaw = [out.fieldtwistangularz out.time];
%Dibujo
figure; 
subplot 221; title('Eje X'); hold on; xlabel('Tiempo'); ylabel('Velocidad')
plot(VinX(:,2),VinX(:,1))
plot(VoutX(:,2),VoutX(:,1))
subplot 222; title('Eje Y'); hold on; xlabel('Tiempo'); ylabel('Velocidad')
plot(VinY(:,2),VinY(:,1))
plot(VoutY(:,2),VoutY(:,1))
subplot 223; title('Eje Z'); hold on; xlabel('Tiempo'); ylabel('Velocidad')
plot(VinZ(:,2),VinZ(:,1))
plot(VoutZ(:,2),VoutZ(:,1))
subplot 224; title('Giro'); hold on; xlabel('Tiempo'); ylabel('Velocidad angular')
plot(VinYaw(:,2),VinYaw(:,1))
plot(VoutYaw(:,2),VoutYaw(:,1))