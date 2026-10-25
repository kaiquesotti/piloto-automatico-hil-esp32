% run_mil_export.m
% Roda o modelo Simulink nao_linear_piloto_automatico (MIL) e exporta
% referencia, velocidade, forca e theta para CSV, sem salvar o .slx.
%
% Uso:  matlab -batch "run('run_mil_export.m')"

here = fileparts(mfilename('fullpath'));
cd(here);

mdl = 'nao_linear_piloto_automatico';
out_csv = fullfile(here, '..', 'resultados_HIL', 'resultado_MIL.csv');

% Parametros do modelo (mesmos do .m e do firmware).
% m, g, rho, Cd, A, uw e f precisam ser GLOBAIS: Modelo_nao_linear.m
% declara global e o bloco MATLAB Function le esses valores.
global m g rho Cd A uw f
m = 1000; g = 9.81; rho = 1.202; A = 1.0; Cd = 0.5; uw = 2.0; f = 0.015;
Ka = 10.0; TauA = 0.2;

if bdIsLoaded(mdl)
    close_system(mdl, 0);   % descarta qualquer alteracao em memoria
end
load_system(mdl);

% Remove blocos de log de execucoes anteriores, se existirem
names = {'log_r', 'log_u', 'log_fx', 'log_theta'};
for k = 1:numel(names)
    blk = [mdl '/' names{k}];
    if ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', names{k}))
        delete_block(blk);
    end
end

% Cria blocos To Workspace (somente em memoria; o .slx nao e salvo)
add_block('simulink/Sinks/To Workspace', [mdl '/log_r'], ...
    'VariableName', 'mil_r', 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', 'Position', [0 500 30 530]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_u'], ...
    'VariableName', 'mil_u', 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', 'Position', [700 500 730 530]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_fx'], ...
    'VariableName', 'mil_fx', 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', 'Position', [320 500 350 530]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_theta'], ...
    'VariableName', 'mil_theta', 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', 'Position', [230 500 260 530]);

% Liga cada bloco ao sinal correspondente
link = @(src, dst) add_line(mdl, ...
    get_param(src, 'PortHandles').Outport(1), ...
    get_param(dst, 'PortHandles').Inport(1), 'autorouting', 'on');
link([mdl '/Step'], [mdl '/log_r']);
link([mdl '/Integrator'], [mdl '/log_u']);
link([mdl '/Transfer Fcn'], [mdl '/log_fx']);
link([mdl '/Signal Editor'], [mdl '/log_theta']);

set_param(mdl, 'StopTime', '200', 'SaveTime', 'on', ...
    'MaxStep', '0.01', 'ReturnWorkspaceOutputs', 'on');

out = sim(mdl);

% Recupera as series (To Workspace grava na base ou em out)
getts = @(name) local_getts(out, name);
tr = getts('mil_r');
tu = getts('mil_u');
tf = getts('mil_fx');
tt = getts('mil_theta');

% Grade comum: usa o tempo da referencia e interpola os demais
t = tr.Time(:);
r = tr.Data(:);
u = interp1(tu.Time(:), tu.Data(:), t, 'linear', 'extrap');
fx = interp1(tf.Time(:), tf.Data(:), t, 'linear', 'extrap');
theta = interp1(tt.Time(:), tt.Data(:), t, 'linear', 'extrap');

fid = fopen(out_csv, 'w');
fprintf(fid, 'time_s,reference_kmh,speed_mps,fx_N,theta_deg\n');
for i = 1:numel(t)
    fprintf(fid, '%.9f,%.9f,%.9f,%.9f,%.9f\n', ...
        t(i), r(i), u(i), fx(i), theta(i));
end
fclose(fid);

fprintf('MIL exportado: %s (%d amostras)\n', out_csv, numel(t));
fprintf('u final = %.4f m/s, Fx final = %.2f N\n', u(end), fx(end));

close_system(mdl, 0);   % descarta alteracoes no modelo

function ts = local_getts(out, name)
    if isa(out, 'Simulink.SimulationOutput') && any(strcmp(out.who, name))
        ts = out.get(name);
    else
        ts = evalin('base', name);
    end
    if isa(ts, 'timeseries')
        return;
    end
    % Caso SaveFormat nao seja Timeseries
    ts = timeseries(ts.Data, ts.Time);
end
