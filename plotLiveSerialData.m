close all

ser = serial('/dev/tty.usbmodemfa131', 'BaudRate', 115200);

fopen(ser);
tic
hold on
grid on
t = toc;
tv = [0 t];
vv = [0 0];
h = plot(tv,vv,'linewidth',3);
hveclen = 300;
hvec = zeros(1,hveclen);
hvec(end) = h;
while(1)
    if strcmp(get(gcf,'currentcharacter'),'q')
        break
    end
    fprintf(ser, '*IDN?');
    idn = str2double(fscanf(ser));
    vv = [vv(2) idn];
    t = toc;
    tv = [tv(2) t];
    if ~isempty(idn)
        h = plot(tv,vv,'linewidth',3);
        if hvec(1)~=0
            delete(hvec(1));
        end
        hvec(1:hveclen-1) = hvec(2:end);
        hvec(end) = h;
        xlim([t-10 t])
        drawnow
    end
end

fclose(ser);
close all