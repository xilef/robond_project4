#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImageNode
{
public:
	ProcessImageNode()
	{
		/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
			ros::console::notifyLoggerLevelsChanged();
		}*/
		ROS_INFO_STREAM("Setup ball chasing service");

		m_client = m_hNode.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

		m_subImage = m_hNode.subscribe("/camera/rgb/image_raw", 30, &ProcessImageNode::processImageCallback, this);

		std::string strNodeName = ros::this_node::getName();

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
	}

private:
	ros::NodeHandle 	m_hNode;
	ros::Subscriber 	m_subImage;
	ros::ServiceClient 	m_client;

	unsigned int		m_uiPrevBallPos;

	unsigned int 		m_uiMaxBallWidth;
	unsigned int		m_uiMaxBallWidthVariance;

	unsigned int		m_uiFastForwardMaxDistance;

	float			m_fFastForwardSpeed;
	float			m_fSlowForwardSpeed;
	float			m_fBackwardSpeed;

	float			m_fMaxRotateSpeed;
	float			m_fMinRotateSpeed;
	float			m_fFindRotateSpeed;

	bool			m_bBallFound;

	unsigned int		m_uiRotateStep;
	unsigned int		m_uiMaxRotateStep;

	float			m_fMaxSideRegionSizeFactor;
	float			m_fMinSideRegionSizeFactor;


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
			m_bBallFound = true;

			if (uiWidth < m_uiMaxBallWidth)
			{
				// Only move forward if seeing the full ball
				if (!bPartialBallFound)
				{
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

			if (uiBallPos < uiLeftRegionEnd)
			{
				fLocationPercentage = uiBallPos / static_cast<float>(uiSideRegionWidth);
				fLocationPercentage = std::min(std::max(fLocationPercentage, 0.0f), 1.0f);
				if (fLocationPercentage < m_fMinSideRegionSizeFactor)
				{
					fLinX = 0.0f;
				}
				fAngZ = m_fMinRotateSpeed;
				//ROS_DEBUG("fLocationPercentage: %1.2f %1.2f fLinX: %1.2f fAngZ: %1.2f", fLocationPercentage, (m_fMinSideRegionSizeFactor), fLinX, fAngZ);
			}
			else if (uiBallPos > uiRightRegionStart)
			{
				fLocationPercentage = (uiBallPos - uiRightRegionStart) / static_cast<float>(uiSideRegionWidth);
				fLocationPercentage = std::min(std::max(fLocationPercentage, 0.0f), 1.0f);
				if (fLocationPercentage < (1.0f - m_fMinSideRegionSizeFactor))
				{
					fLinX = 0.0f;
				}
				fAngZ = -(m_fMinRotateSpeed);
				//ROS_DEBUG("fLocationPercentage: %1.2f %1.2f fLinX: %1.2f fAngZ: %1.2f", fLocationPercentage, (1.0f - m_fMinSideRegionSizeFactor), fLinX, fAngZ);
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
		m_uiPrevBallPos = uiBallPos;
		driveRobot(fLinX, fAngZ);
	}

	// The whole logic does not handle yet obstacles partially obstructing the ball view
	// also have not handled yet the robot itself being obstructed
	void processImageCallback(const sensor_msgs::Image img)
	{
		unsigned int uiWhitePixel = 255;
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
					// This will have a problem if there are multiple white balls in the viewport
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
			// Also get the ball position
			if (uiCurrentWidth > uiWidth)
			{
				uiWidth = uiCurrentWidth;
				uiBallPos = uiFirstWhitePixelPos + (uiWidth / 2);
			}

			if (uiCurrentWidth == 0 && bBallFound)
			{
				// Assume no more part of the ball to be found
				break;
			}
		}

		determineMovement(uiWidth, uiBallPos, bPartialBallFound, img.width);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "process_image");

	ProcessImageNode obj;

	ros::spin();

	return 0;
}
