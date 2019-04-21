#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

typedef struct pixel
{
	unsigned int uiRed;
	unsigned int uiGreen;
	unsigned int uiBlue;

	pixel()
	{
		reset();
	}

	bool operator==(const pixel& param) const
	{
		return this->uiRed == param.uiRed &&
				this->uiGreen == param.uiGreen &&
				this->uiBlue == param.uiBlue;
	}
	
	bool operator>(const pixel& param) const
	{
		return this->uiRed > param.uiRed &&
				this->uiGreen > param.uiGreen &&
				this->uiBlue > param.uiBlue;
	}
	
	bool operator<(const pixel& param) const
	{
		return this->uiRed < param.uiRed &&
				this->uiGreen < param.uiGreen &&
				this->uiBlue < param.uiBlue;
	}

	pixel operator=(const pixel& param)
	{
		this->uiRed = param.uiRed;
		this->uiGreen = param.uiGreen;
		this->uiBlue = param.uiBlue;
		return *this;
	}

	void set(unsigned int red, unsigned int green, unsigned int blue)
	{
		uiRed = red;
		uiGreen = green;
		uiBlue = blue;
	}

	void reset()
	{
		uiRed = 256;
		uiGreen = 256;
		uiBlue = 256;
	}

	bool isNotSet()
	{
		return uiRed == 256 &&
				uiGreen == 256 &&
				uiBlue == 256;
	}

} pixel;

typedef struct ballData
{
	pixel pxColor;
	unsigned int uiWidth;
	unsigned int uiPos;
	bool bPartial;
	
	ballData()
	{
		reset();
	}
	
	void reset()
	{
		pxColor.reset();
		uiWidth = 0;
		uiPos = 0;
		bPartial = false;
	}
} ballData;

class ProcessImageNode
{
public:
	ProcessImageNode()
	{
		/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
			ros::console::notifyLoggerLevelsChanged();
		}*/

		ROS_INFO_STREAM("Setup ball chasing node");

		m_client = m_hNode.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

		m_subImage = m_hNode.subscribe("/camera/rgb/image_raw", 30, &ProcessImageNode::processImageCallback, this);

		std::string strNodeName = ros::this_node::getName();

		// Init ball finding vars
		int iMinBallDetectWidth;
		m_hNode.getParam(strNodeName + "/minBallDetectWidth", iMinBallDetectWidth);
		m_uiMinBallDetectWidth = std::min(std::max(iMinBallDetectWidth, 0), UINT8_MAX);
	
		int iHWTolerance;
		m_hNode.getParam(strNodeName + "/heightWidthTolerance", iHWTolerance);
		m_uiHWTolerance = std::min(std::max(iHWTolerance, 0), UINT8_MAX);

		int iSameWTolerance;
		m_hNode.getParam(strNodeName + "/sameWidthTolerance", iSameWTolerance);
		m_uiSameWTolerance = std::min(std::max(iSameWTolerance, 0), UINT8_MAX);

		int iMaxWidthFoundTolerance;
		m_hNode.getParam(strNodeName + "/maxWidthFoundTolerance", iMaxWidthFoundTolerance);
		m_uiMaxWidthFoundTolerance = std::min(std::max(iMaxWidthFoundTolerance, 0), UINT8_MAX);


		// Init moving vars
		int iMaxBallWidth;
		m_hNode.getParam(strNodeName + "/maxBallWidth", iMaxBallWidth);
		m_uiMaxBallWidth = std::min(std::max(iMaxBallWidth, 1), UINT8_MAX);

		int iMaxBallWidthVariance;
		m_hNode.getParam(strNodeName + "/maxBallWidthVariance", iMaxBallWidthVariance);
		m_uiMaxBallWidthVariance = std::min(std::max(iMaxBallWidthVariance, 1), UINT8_MAX);

		int iFastForwardMaxDistance;
		m_hNode.getParam(strNodeName + "/fastForwardMaxDistance", iFastForwardMaxDistance);
		m_uiFastForwardMaxDistance = std::min(std::max(iFastForwardMaxDistance, 1), UINT8_MAX);

		float fMaxForwardSpeed;
		m_hNode.getParam(strNodeName + "/maxForwardSpeed", fMaxForwardSpeed);
		fMaxForwardSpeed = std::min(std::max(fMaxForwardSpeed, 0.1f), 5.0f);

		m_hNode.getParam(strNodeName + "/fastForwardSpeed", m_fFastForwardSpeed);
		m_fFastForwardSpeed = std::min(std::max(m_fFastForwardSpeed, 0.1f), fMaxForwardSpeed);

		m_hNode.getParam(strNodeName + "/slowForwardSpeed", m_fSlowForwardSpeed);
		m_fSlowForwardSpeed = std::min(std::max(m_fSlowForwardSpeed, 0.1f), m_fFastForwardSpeed);

		float fMaxBackwardSpeed;
		m_hNode.getParam(strNodeName + "/maxBackwardSpeed", fMaxBackwardSpeed);
		fMaxBackwardSpeed = std::min(std::max(fMaxBackwardSpeed, 0.1f), 5.0f);

		m_hNode.getParam(strNodeName + "/backwardSpeed", m_fBackwardSpeed);
		m_fBackwardSpeed = std::min(std::max(m_fBackwardSpeed, 0.1f), fMaxBackwardSpeed);
		if (m_fBackwardSpeed > 0)
		{
			m_fBackwardSpeed *= -1;
		}

		float fMaxOverallRotateSpeed;
		m_hNode.getParam(strNodeName + "/maxOverallRotateSpeed", fMaxOverallRotateSpeed);
		fMaxOverallRotateSpeed = std::min(std::max(fMaxOverallRotateSpeed, 0.1f), 5.0f);

		m_hNode.getParam(strNodeName + "/maxRotateSpeed", m_fMaxRotateSpeed);
		m_fMaxRotateSpeed = std::min(std::max(m_fMaxRotateSpeed, 0.1f), fMaxOverallRotateSpeed);

		m_hNode.getParam(strNodeName + "/minRotateSpeed", m_fMinRotateSpeed);
		m_fMinRotateSpeed = std::min(std::max(m_fMinRotateSpeed, 0.1f), m_fMaxRotateSpeed);

		m_hNode.getParam(strNodeName + "/findRotateSpeed", m_fFindRotateSpeed);
		m_fFindRotateSpeed = std::min(std::max(m_fFindRotateSpeed, -(fMaxOverallRotateSpeed)), fMaxOverallRotateSpeed);

		m_hNode.getParam(strNodeName + "/maxSideRegionSizeFactor", m_fMaxSideRegionSizeFactor);
		m_fMaxSideRegionSizeFactor = std::min(std::max(m_fMaxSideRegionSizeFactor, 0.2f), 0.99f);

		m_hNode.getParam(strNodeName + "/minSideRegionSizeFactor", m_fMinSideRegionSizeFactor);
		m_fMinSideRegionSizeFactor = std::min(std::max(m_fMinSideRegionSizeFactor, 0.1f), m_fMaxSideRegionSizeFactor);

		int iMaxRotateStep;
		m_hNode.getParam(strNodeName + "/maxRotateStep", iMaxRotateStep);
		m_uiMaxRotateStep = std::min(std::max(iMaxRotateStep, 1), UINT8_MAX);

		m_bBallFound = false;
		m_uiRotateStep = 0;

		ROS_INFO_STREAM("Ready to chase balls!");
	}

private:
	ros::NodeHandle 	m_hNode;
	ros::Subscriber 	m_subImage;
	ros::ServiceClient 	m_client;

	// Ball finding vars
	unsigned int		m_uiMinBallDetectWidth;
	unsigned int		m_uiHWTolerance;
	unsigned int		m_uiSameWTolerance;
	unsigned int		m_uiMaxWidthFoundTolerance;

	// Robot movement vars
	unsigned int 		m_uiMaxBallWidth;
	unsigned int		m_uiMaxBallWidthVariance;

	unsigned int		m_uiFastForwardMaxDistance;

	float				m_fFastForwardSpeed;
	float				m_fSlowForwardSpeed;
	float				m_fBackwardSpeed;

	float				m_fMaxRotateSpeed;
	float				m_fMinRotateSpeed;
	float				m_fFindRotateSpeed;

	unsigned int		m_uiRotateStep;
	unsigned int		m_uiMaxRotateStep;

	float				m_fMaxSideRegionSizeFactor;
	float				m_fMinSideRegionSizeFactor;

	bool				m_bBallFound;
	ballData			m_bdCurrBall;


	void driveRobot(float fLinX, float fAngz)
	{
		ball_chaser::DriveToTarget req;

		req.request.linear_x = fLinX;
		req.request.angular_z = fAngz;

		if (!m_client.call(req))
		{
			ROS_ERROR("Failed to call service!");
		}
	}

	void determineMovement(unsigned int uiWidth, unsigned int uiBallPos, bool bPartialBallFound, unsigned int uiImgWidth)
	{
		// Declare regions based on the camera image width
		unsigned int uiSideRegionWidth = ((uiImgWidth / 2) * m_fMaxSideRegionSizeFactor);
		unsigned int uiLeftRegionEnd = uiSideRegionWidth;
		unsigned int uiRightRegionStart = uiImgWidth - uiSideRegionWidth;

		// For the drive request
		float fLinX = 0.0f;
		float fAngZ = 0.0f;

		// Can see a ball
		if (uiWidth > 0)
		{
			m_uiRotateStep = 0;

			// Don't go too near the ball
			// remove for rocket league
			if (uiWidth < m_uiMaxBallWidth)
			{
				// Only move forward if seeing the full ball
				if (!bPartialBallFound)
				{
					// Slowdown when near
					// remove for rocket league
					if (m_uiMaxBallWidth - uiWidth > m_uiFastForwardMaxDistance)
					{
						fLinX = m_fFastForwardSpeed;
					}
					else
					{
						fLinX = m_fSlowForwardSpeed;
					}
				}
			}
			else if (uiWidth > m_uiMaxBallWidth + m_uiMaxBallWidthVariance)
			{
				fLinX = m_fBackwardSpeed;
			}

			float fLocationPercentage;
			float fBallSizeFactor = uiWidth / static_cast<float>(m_uiMaxBallWidth);
			fBallSizeFactor = std::min(std::max(fBallSizeFactor, 0.1f), 1.0f);

			// Adjust turning speed based on the ball distance
			// slow when near to prevent overshoots
			float fAdjustedRotateSpeed = m_fMaxRotateSpeed - ((m_fMaxRotateSpeed - m_fMinRotateSpeed) * fBallSizeFactor);

			if (uiBallPos < uiLeftRegionEnd)
			{
				// Turn only when the ball is far from the centre region
				// to try to reduce course corrections along the way
				fLocationPercentage = uiBallPos / static_cast<float>(uiSideRegionWidth);
				fLocationPercentage = std::min(std::max(fLocationPercentage, 0.0f), 1.0f);
				if (fLocationPercentage < m_fMinSideRegionSizeFactor)
				{
					fLinX = 0.0f;
				}

				fAngZ = fAdjustedRotateSpeed;
			}
			else if (uiBallPos > uiRightRegionStart)
			{
				// Turn only when the ball is far from the centre region
				// to try to reduce course corrections along the way
				fLocationPercentage = (uiBallPos - uiRightRegionStart) / static_cast<float>(uiSideRegionWidth);
				fLocationPercentage = std::min(std::max(fLocationPercentage, 0.0f), 1.0f);
				if (fLocationPercentage < m_fMinSideRegionSizeFactor)
				{
					fLinX = 0.0f;
				}

				fAngZ = -(fAdjustedRotateSpeed);
			}
		}
		// Ball is playing hide and seek, try to roughly turn around once to find it
		else
		{
			if (m_bBallFound)
			{
				if (m_uiRotateStep >= m_uiMaxRotateStep)
				{
					m_bBallFound = false;
					m_bdCurrBall.reset();
					m_uiRotateStep = 0;
					m_fFindRotateSpeed *= -1;
				}
				else
				{
					m_uiRotateStep++;
					fAngZ = m_fFindRotateSpeed;
				}
			}
		}

		driveRobot(fLinX, fAngZ);
	}

	// This is most definitely the worst logic for finding a ball
	bool findBall(const sensor_msgs::Image& img, pixel pxPrevColor, unsigned int uiStartPos, unsigned int uiStartWidth, unsigned int uiCurrRow)
	{
		//ROS_DEBUG("findBall RGB:%3d%3d%3d sp:%d sw:%d cr:%d", pxPrevColor.uiRed, pxPrevColor.uiGreen, pxPrevColor.uiBlue, uiStartPos, uiStartWidth, uiCurrRow);
		bool bRet = false;

		pixel pxCurrColor;

		unsigned int uiH = 0;
		unsigned int uiW = 0;

		unsigned int uiLeftWidth = 0;
		unsigned int uiRightWidth = 0;
		unsigned int uiWidth = uiStartWidth;
		unsigned int uiCurrWidth = 0;
		unsigned int uiSameWidth = 0;
		unsigned int uiBlockedWidth = 0;

		bool bMaxWidthFound = false;
		unsigned int uiMaxWidthFoundError = 0;

		unsigned int uiHeight = 0;

		unsigned int uiMidPos = uiStartPos + (uiStartWidth / 2);

		bool bPartialBallFound = false;

		for (uiH = uiCurrRow; uiH < img.height; uiH++)
		{
			uiLeftWidth = 0;
			uiRightWidth = 0;
			uiBlockedWidth = 0;

			for (uiW = uiMidPos * 3; uiW > 0; uiW -= 3)
			{
				pxCurrColor.set(img.data[uiW + (uiH * img.step)],
							img.data[uiW + (uiH * img.step) + 1],
							img.data[uiW + (uiH * img.step) + 2]);

				if (pxCurrColor == pxPrevColor)
				{
					uiLeftWidth++;
					uiLeftWidth += uiBlockedWidth;
					uiBlockedWidth = 0;
				}
				else
				{
					uiBlockedWidth++;
				}
			}
			if (uiW == 0)
			{
				pxCurrColor.set(img.data[uiW + (uiH * img.step)],
							img.data[uiW + (uiH * img.step) + 1],
							img.data[uiW + (uiH * img.step) + 2]);

				// Same color at the edge, might not be seeing the full ball yet
				if (pxCurrColor == pxPrevColor)
				{
					bPartialBallFound = true;
				}
			}

			uiBlockedWidth = 0;
			for (uiW = (uiMidPos + 1) * 3; uiW < img.step; uiW += 3)
			{
				pxCurrColor.set(img.data[uiW + (uiH * img.step)],
							img.data[uiW + (uiH * img.step) + 1],
							img.data[uiW + (uiH * img.step) + 2]);

				if (pxCurrColor == pxPrevColor)
				{
					uiRightWidth++;
					uiRightWidth += uiBlockedWidth;
					uiBlockedWidth = 0;
				}
				else
				{
					uiBlockedWidth++;
				}
			}
			if (uiW == img.step - 3)
			{
				pxCurrColor.set(img.data[uiW + (uiH * img.step)],
							img.data[uiW + (uiH * img.step) + 1],
							img.data[uiW + (uiH * img.step) + 2]);

				// Same color at the edge, might not be seeing the full ball yet
				if (pxCurrColor == pxPrevColor)
				{
					bPartialBallFound = true;
				}
			}

			if (m_bdCurrBall.pxColor.isNotSet())
			{
				//ROS_DEBUG("curh:%d h:%d lw:%d rw:%d w:%d same:%d part:%d max:%d", uiH, uiHeight, uiLeftWidth, uiRightWidth, uiWidth, uiSameWidth, bPartialBallFound, bMaxWidthFound);
				// Only detect if the full ball is seen if there is currently no ball being tracked
				if (uiLeftWidth > 0 && uiRightWidth > 0 && !bPartialBallFound)
				{
					uiCurrWidth = uiLeftWidth + uiRightWidth;
					uiHeight++;

					if (uiCurrWidth > uiWidth)
					{
						uiSameWidth = 0;
						uiWidth = uiCurrWidth;
						uiMidPos = (uiMidPos - uiLeftWidth) + (uiCurrWidth / 2);

						// Might be a camera artifact if the current line is suddenly more than the supposed max
						if (bMaxWidthFound)
						{
							uiMaxWidthFoundError++;
							if (uiMaxWidthFoundError > m_uiMaxWidthFoundTolerance)
							{
								bMaxWidthFound = false;
								break;
							}
						}
					}
					else if (uiCurrWidth < uiWidth)
					{
						uiSameWidth = 0;
						bMaxWidthFound = true;
					}
					else
					{
						uiSameWidth++;
						// Might be something else if still the same width
						// or not due to camera
						if (uiSameWidth >= m_uiSameWTolerance)
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			else
			{
				uiCurrWidth = uiLeftWidth + uiRightWidth;
				if (uiCurrWidth > 0)
				{
					uiHeight++;
				}

				if (uiCurrWidth > uiWidth)
				{
					uiWidth = uiCurrWidth;
					uiMidPos = (uiMidPos - uiLeftWidth) + (uiCurrWidth / 2);
				}
			}
		}

		if (m_bdCurrBall.pxColor.isNotSet())
		{
			// Only detect if a ball if within a certain size
			if (uiWidth > m_uiMinBallDetectWidth && uiHeight > m_uiMinBallDetectWidth)
			{
				// Should be a ball if:
				// height is almost equal to width, width is increasing at the top and decreasing at the bottom
				// and there is not too much same width
				if (abs(uiHeight - uiWidth) < m_uiHWTolerance && bMaxWidthFound && uiSameWidth < m_uiSameWTolerance)
				{
					m_bBallFound = true;
					bRet = true;
				}
			}
		}
		else
		{
			if (uiWidth > 0)
			{
				bRet = true;
			}
		}

		if (m_bBallFound)
		{
			//ROS_DEBUG("RGB:%3d%3d%3d width:%d pos:%d partial:%d", pxPrevColor.uiRed, pxPrevColor.uiGreen, pxPrevColor.uiBlue, uiWidth, uiMidPos, bPartialBallFound);
			m_bdCurrBall.pxColor = pxPrevColor;
			m_bdCurrBall.uiWidth = uiWidth;
			m_bdCurrBall.uiPos = uiMidPos;
			m_bdCurrBall.bPartial = bPartialBallFound;
		}
		
		return bRet;
	}

	// The whole logic does not handle yet obstacles partially obstructing the ball view
	// also have not handled yet the robot itself being obstructed
	void processImageCallback(const sensor_msgs::Image img)
	{
		bool bFound = false;
		unsigned int uiWidth = 0;
		pixel pxCurrColor;
		pixel pxPrevColor = m_bdCurrBall.pxColor;

		pixel pxSkyColor;
		pxSkyColor.set(178, 178, 178);

		pixel pxStartShadowColor;
		pxStartShadowColor.set(90, 80, 60);
		pixel pxEndShadowColor;
		pxEndShadowColor.set(110, 100, 80);
		
		pixel pxShadowColor; // looks like a weird shadow of the whole scene
		pxShadowColor.set(163, 152, 132);
		/*pixel pxShadowColor2; // uh
		pxShadowColor2.set(108, 96, 77);
		pixel pxShadowColor3; // uh
		pxShadowColor3.set(94, 82, 63);*/

		//ROS_DEBUG("newProcessImageCallback RGB:%3d%3d%3d", pxPrevColor.uiRed, pxPrevColor.uiGreen, pxPrevColor.uiBlue);

		unsigned int uiH = 0;
		unsigned int uiW = 0;

		unsigned int uiStartPos = 0;

		for (uiH = 0; uiH < img.height && !bFound; uiH++)
		{
			uiWidth = 0;
			uiStartPos = 0;

			for (uiW = 0; uiW < img.step && !bFound; uiW += 3)
			{
				pxCurrColor.set(img.data[uiW + (uiH * img.step)],
							img.data[uiW + (uiH * img.step) + 1],
							img.data[uiW + (uiH * img.step) + 2]);

				// Do not track the color of the sky and some shadows and some uh
				if (pxCurrColor == pxSkyColor || pxCurrColor == pxShadowColor || (pxCurrColor > pxStartShadowColor && pxCurrColor < pxEndShadowColor))
				{
					continue;
				}

				if (pxCurrColor == pxPrevColor)
				{
					uiWidth++;

					if (uiStartPos == 0)
					{
						uiStartPos = uiW / 3;
					}
				}
				else
				{
					if (m_bdCurrBall.pxColor.isNotSet())
					{
						if (uiWidth > m_uiMinBallDetectWidth)
						{
							bFound = findBall(img, pxPrevColor, uiStartPos, uiWidth, uiH);
						}
					}
					else
					{
						if (uiWidth > 0)
						{
							bFound = findBall(img, pxPrevColor, uiStartPos, uiWidth, uiH);
						}
					}

					uiWidth = 0;
					uiStartPos = 0;
				}

				if (!m_bBallFound)
				{
					pxPrevColor = pxCurrColor;
				}
			}
		}
		
		if (m_bBallFound)
		{
			if (!bFound)
			{
				m_bdCurrBall.uiWidth = 0;
			}
		}

		//ROS_DEBUG("w:%d pos:%d partial:%d imgw:%d", m_bdCurrBall.uiWidth, m_bdCurrBall.uiPos, m_bdCurrBall.bPartial, img.width);
		determineMovement(m_bdCurrBall.uiWidth, m_bdCurrBall.uiPos, m_bdCurrBall.bPartial, img.width);
	}

	/*void processImageCallback(const sensor_msgs::Image img)
	{
		ROS_DEBUG("processImageCallback");
		const unsigned int uiWhitePixel = 255;

		unsigned int uiCurrentWidth;
		unsigned int uiBlockedWidth;
		unsigned int uiWidth = 0;
		unsigned int uiBallPos = 0;
		unsigned int uiFirstWhitePixelPos;
		unsigned int uiH = 0;
		unsigned int uiW = 0;

		unsigned int uiOffset = img.step / img.width;

		bool bBallFound = false;
		bool bPartialBallFound = false;

		for (uiH = 0; uiH < img.height; uiH++)
		{
			uiCurrentWidth = 0;
			uiBlockedWidth = 0;
			uiFirstWhitePixelPos = img.step + 1;

			for (uiW = 0; uiW < img.step; uiW += uiOffset)
			{
				// Track how wide the ball is and where it is
				if (img.data[uiW + (img.step * uiH)] == uiWhitePixel)
				{
					// Can only see a partial of the ball if the edges are white
					if (uiW == 0 || uiW == img.step - uiOffset)
					{
						bPartialBallFound = true;
					}
					bBallFound = true;

					uiCurrentWidth++;

					// Add back the blocked width if there is
					// this will have a problem if there are multiple white balls in the viewport
					if (uiBlockedWidth > 0)
					{
						uiCurrentWidth += uiBlockedWidth;
						uiBlockedWidth = 0;
					}

					// Save it for later ball pos calculation
					if (uiFirstWhitePixelPos == img.step + 1)
					{
						uiFirstWhitePixelPos = uiW / uiOffset;
					}
				}
				else
				{
					// Might be blocked by something
					if (uiCurrentWidth > 0)
					{
						uiBlockedWidth++;
					}
				}
			}

			// Find the current width to determine if we need to move forward
			// also get the ball position
			if (uiCurrentWidth > uiWidth)
			{
				uiWidth = uiCurrentWidth;
				uiBallPos = uiFirstWhitePixelPos + (uiWidth / 2);
			}

			// Assume no more part of the ball to be found
			// do not waste any more time and immediately exit
			// for future: can probably exit when the width has been found
			if (uiCurrentWidth == 0 && bBallFound)
			{
				break;
			}
		}

		//determineMovement(uiWidth, uiBallPos, bPartialBallFound, img.width);
	}*/
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "process_image");

	ProcessImageNode obj;

	ros::spin();

	return 0;
}
