function [Jreg, gradP, details] = regularizationCostGrad2D(P, Pref, params)
%REGULARIZATIONCOSTGRAD2D Reference, smoothness, and length costs/gradients.

    n = size(P,1);
    gradP = zeros(size(P));

    wRef = getParam(params, 'wRef', 0);
    wSmooth = getParam(params, 'wSmooth', 0);
    wLength = getParam(params, 'wLength', 0);
    wTrust = getParam(params, 'wTrust', 0);

    Jref = 0;
    if wRef > 0
        D = P - Pref;
        Jref = wRef * sum(sum(D.^2));
        gradP = gradP + 2*wRef*D;
    end

    Jtrust = 0;
    if wTrust > 0 && isfield(params, 'PbaseTrust') && ~isempty(params.PbaseTrust)
        D = P - params.PbaseTrust;
        Jtrust = wTrust * sum(sum(D.^2));
        gradP = gradP + 2*wTrust*D;
    end

    Jsmooth = 0;
    if wSmooth > 0
        for i = 2:n-1
            S = P(i-1,:) - 2*P(i,:) + P(i+1,:);
            Jsmooth = Jsmooth + wSmooth * dot(S,S);
            gradP(i-1,:) = gradP(i-1,:) + 2*wSmooth*S;
            gradP(i,:)   = gradP(i,:)   - 4*wSmooth*S;
            gradP(i+1,:) = gradP(i+1,:) + 2*wSmooth*S;
        end
    end

    Jlen = 0;
    if wLength > 0
        for i = 1:n-1
            e = P(i+1,:) - P(i,:);
            le = norm(e);
            if le < 1e-12
                continue;
            end
            Jlen = Jlen + wLength * le;
            dir = e / le;
            gradP(i,:) = gradP(i,:) - wLength * dir;
            gradP(i+1,:) = gradP(i+1,:) + wLength * dir;
        end
    end

    Jreg = Jref + Jsmooth + Jlen + Jtrust;

    details = struct();
    details.Jref = Jref;
    details.Jsmooth = Jsmooth;
    details.Jlen = Jlen;
    details.Jtrust = Jtrust;
end

function val = getParam(params, name, defaultVal)
    if isfield(params, name)
        val = params.(name);
    else
        val = defaultVal;
    end
end
