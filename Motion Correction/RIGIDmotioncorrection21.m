Bmode = 'bmodetrim.mp4';
CEUS = '115601_CEUS.mp4'; 
BmodeOutput = '115601_Bmode_rigid_stabilized.mp4';
CEUSOutput = '115601_CEUS_rigid_stabilized.mp4';

stabilizeDualVideo(Bmode, CEUS, BmodeOutput, CEUSOutput);

function stabilizeDualVideo(Bmode, CEUS, BmodeOutput, CEUSOutput)
    % Initialize Readers
    v1 = VideoReader(Bmode);
    v2 = VideoReader(CEUS);
    
    % Initialize Writers
    vOut1 = VideoWriter(BmodeOutput, 'MPEG-4');
    vOut2 = VideoWriter(CEUSOutput, 'MPEG-4');
    vOut1.FrameRate = v1.FrameRate;
    vOut2.FrameRate = v1.FrameRate;
    
    open(vOut1); open(vOut2);
    
    % --- 1. SETUP ROI & REFERENCE ---
    firstFrame = readFrame(v1);
    imshow(firstFrame);
    title('Select the Region of Interest (ROI) to track, then double-click.');
    roiRect = round(getrect()); % [xmin ymin width height]
    
    % Define the cropping indices for the ROI
    rIdx = roiRect(2):(roiRect(2) + roiRect(4));
    cIdx = roiRect(1):(roiRect(1) + roiRect(3));
    
    % Extract and process the reference ROI
    refROI = im2gray(firstFrame(rIdx, cIdx, :));
    [rowsROI, colsROI] = size(refROI);
    win = hanning(rowsROI) * hanning(colsROI)';
    fftRef = fft2(double(refROI) .* win);
    
    % --- 2. CALCULATE SHIFTS (BASED ON ROI) ---
    totalFrames = floor(v1.Duration * v1.FrameRate);
    shifts = zeros(totalFrames, 2);
    v1.CurrentTime = 0;
    
    fprintf('Analyzing motion in ROI...\n');
    for i = 1:totalFrames
        if ~hasFrame(v1), break; end
        % Get the ROI from the current frame
        currFull = im2gray(readFrame(v1));
        currROI = double(currFull(rIdx, cIdx));
        
        % Phase Correlation on ROI only
        fftCurr = fft2(currROI .* win);
        R = (fftRef .* conj(fftCurr)) ./ (abs(fftRef .* conj(fftCurr)) + eps);
        shiftMap = real(ifft2(R));
        
        [~, maxIdx] = max(shiftMap(:));
        [rp, cp] = ind2sub([rowsROI, colsROI], maxIdx);
        
        % Sub-pixel Estimation
        rRange = mod(rp-2:rp, rowsROI) + 1;
        cRange = mod(cp-2:cp, colsROI) + 1;
        patch = shiftMap(rRange, cRange);
        y = patch(:, 2); x = patch(2, :);
        
        dr_sub = 0; dc_sub = 0;
        if y(1) ~= y(3), dr_sub = (y(3) - y(1)) / (2 * (2 * y(2) - y(1) - y(3))); end
        if x(1) ~= x(3), dc_sub = (x(3) - x(1)) / (2 * (2 * x(2) - x(1) - x(3))); end

        dR = (rp - 1 + dr_sub); if dR > rowsROI/2, dR = dR - rowsROI; end
        dC = (cp - 1 + dc_sub); if dC > colsROI/2, dC = dC - colsROI; end
        
        shifts(i, :) = [dC, dR];
    end
    
    % --- 3. APPLY TO BOTH VIDEOS ---
    % Auto-crop calculation based on full frame dimensions
    [fRows, fCols, ~] = size(firstFrame);
    maxExcursion = ceil(max(abs(shifts(:))));
    cropMargin = maxExcursion + 2; 
    fullRect = [cropMargin, cropMargin, fCols - 2*cropMargin, fRows - 2*cropMargin];
    
    v1.CurrentTime = 0;
    v2.CurrentTime = 0;
    
    fprintf('Applying stabilization to both videos...\n');
    for i = 1:totalFrames
        if ~hasFrame(v1) || ~hasFrame(v2), break; end
        
        % Read frames from both streams
        f1 = readFrame(v1);
        f2 = readFrame(v2);
        
        % Apply the SAME shift to both
        corr1 = imtranslate(f1, shifts(i, :), 'linear', 'OutputView', 'same');
        corr2 = imtranslate(f2, shifts(i, :), 'linear', 'OutputView', 'same');
        
        % Crop and Write
        writeVideo(vOut1, imcrop(corr1, fullRect));
        writeVideo(vOut2, imcrop(corr2, fullRect));
    end
    
    close(vOut1); close(vOut2);
    fprintf('Rigid Stabilisation Complete!\n');
end

