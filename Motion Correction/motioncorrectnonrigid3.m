
%Input names of input and output videos

Bmode   = '115601_Bmode_rigid_stabilized.mp4';       
CEUS    = '115601_CEUS.mp4';
finalBmode     = '115601_Bmode_stabilized.mp4';
finalCEUS      = '115601_CEUS_stabilized.mp4';

nonRigidStabilizeDualVideo(BmodeFile, CEUSFile, stabilizedBmodeOutputFile, stabilizedCEUSOutputFile)

function nonRigidStabilizeDualVideo(BmodeFile, CEUSFile, stabilizedBmodeOutputFile, stabilizedCEUSOutputFile)
    %% 1. Setup Sources
    vMotion = VideoReader(BmodeFile);
    vTarget = VideoReader(CEUSFile);

    vOutTarget = VideoWriter(stabilizedCEUSOutputFile, 'MPEG-4');
    vOutMotion = VideoWriter(stabilizedBmodeOutputFile, 'MPEG-4');
    vOutTarget.FrameRate = vTarget.FrameRate;
    vOutMotion.FrameRate = vMotion.FrameRate;
    open(vOutTarget); open(vOutMotion);

    %% 2. Process with Dynamic Reference
    % Initialize first reference
    firstFrame = readFrame(vMotion);
    fixedSmooth = single(imgaussfilt(im2gray(firstFrame), 1.5));
    vMotion.CurrentTime = 0;
    frameCount = 0;
    ssimThreshold = 0.82; % Trigger for reference update
    fprintf('Processing frames with Dynamic Reference...\n');

    while hasFrame(vMotion) && hasFrame(vTarget)
        frameCount = frameCount + 1;
        movFrameRaw = readFrame(vMotion);
        tarFrameRaw = readFrame(vTarget);
        movingGray = single(imgaussfilt(im2gray(movFrameRaw), 1.5));

        if ~isequal(size(movingGray), size(fixedSmooth))
            error('Dimension mismatch! Fixed: %s, Moving: %s', mat2str(size(fixedSmooth)), mat2str(size(movingGray)));
        end

        % 1. Estimate Motion
        [D, movingReg] = imregdemons(movingGray, fixedSmooth, [100 50 25], ...
            'AccumulatedFieldSmoothing', 1.3, ...
            'PyramidLevels', 3, ...
            'DisplayWaitbar', false);

        %{
        % 2. Check Quality (SSIM)
        % Compare the registered B-mode to the CURRENT reference
        currentScore = ssim(movingReg, fixedSmooth);
        % 3. Update Reference if quality degrades
        % This prevents the algorithm from "over-warping" to a stale template
        if currentScore < ssimThreshold
            fprintf('SSIM dropped to %.2f. Updating reference at frame %d...\n', currentScore, frameCount);
            fixedSmooth = movingGray; % The current frame becomes the new baseline
            % Re-calculate D for the new reference (should be identity/zero)
            D = zeros(size(D));
            correctedTarget = tarFrameRaw;
            correctedMotion = movFrameRaw;
        else
            % Apply D to both frames
            correctedTarget = imwarp(tarFrameRaw, D);
            correctedMotion = imwarp(movFrameRaw, D);
        end
        %}

        % 2. Apply the calculated displacement to both videos
        correctedTarget = imwarp(tarFrameRaw, D);
        correctedMotion = imwarp(movFrameRaw, D);

        % 3. THE ROLLING UPDATE
        % alpha (0.01 to 0.1) controls how "fast" the memory of the AI is.
        % 0.05 means the reference is 95% previous frames and 5% the new frame.
        alpha = 0.01;
        fixedSmooth = (1 - alpha) * fixedSmooth + alpha * movingGray;

        % 4. Write to disk
        writeVideo(vOutTarget, uint8(correctedTarget));
        writeVideo(vOutMotion, uint8(correctedMotion));

        if mod(frameCount, 20) == 0
            % fprintf('Frame %d | SSIM: %.3f\n', frameCount, currentScore);
        end
    end

    close(vOutTarget); close(vOutMotion);
    fprintf('Finished! Dynamic stabilization complete.\n');
end
