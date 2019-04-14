#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImageNode
{
public:
	ProcessImageNode()
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
			ros::console::notifyLoggerLevelsChanged();
		}

		ROS_INFO_STREAM("Setup ball chasing service");

		m_client = m_hNode.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

		m_subImage = m_hNode.subscribe("/camera/rgb/image_raw", 10, &ProcessImageNode::processImageCallback, this);

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

		float fMaxRotateSpeed;
		m_hNode.getParam(strNodeName + "/maxRotateSpeed", fMaxRotateSpeed);
		fMaxRotateSpeed = std::min(std::max(fMaxRotateSpeed, 0.1f), 5.0f);

		m_hNode.getParam(strNodeName + "/fastRotateSpeed", m_fFastRotateSpeed);
		m_fFastRotateSpeed = std::min(std::max(m_fFastRotateSpeed, 0.1f), fMaxRotateSpeed);

		m_hNode.getParam(strNodeName + "/slowRotateSpeed", m_fSlowRotateSpeed);
		m_fSlowRotateSpeed = std::min(std::max(m_fSlowRotateSpeed, 0.1f), m_fFastRotateSpeed);

		m_hNode.getParam(strNodeName + "/findRotateSpeed", m_fFindRotateSpeed);
		m_fFindRotateSpeed = std::min(std::max(m_fFindRotateSpeed, 0.1f), m_fFastRotateSpeed);

		m_hNode.getParam(strNodeName + "/sideRegionSizeFactor", m_fSideRegionSizeFactor);
		m_fSideRegionSizeFactor = std::min(std::max(m_fSideRegionSizeFactor, 0.1f), 0.9f);

		m_hNode.getParam(strNodeName + "/fastRotateSizeFactor", m_fFastRotateSizeFactor);
		m_fFastRotateSizeFactor = std::min(std::max(m_fFastRotateSizeFactor, 0.1f), 0.9f);

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

	unsigned int 		m_uiMaxBallWidth;
	unsigned int		m_uiMaxBallWidthVariance;

	unsigned int		m_uiFastForwardMaxDistance;

	float			m_fFastForwardSpeed;
	float			m_fSlowForwardSpeed;
	float			m_fBackwardSpeed;

	float			m_fFastRotateSpeed;
	float			m_fSlowRotateSpeed;
	float			m_fFindRotateSpeed;

	bool			m_bBallFound;

	unsigned int		m_uiRotateStep;
	unsigned int		m_uiMaxRotateStep;

	float			m_fSideRegionSizeFactor;

	float			m_fFastRotateSizeFactor;

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

		unsigned int uiSideRegionWidth = ((img.width / 2) * m_fSideRegionSizeFactor);
		unsigned int uiLeftRegionEnd = uiSideRegionWidth;
		unsigned int uiRightRegionStart = img.width - uiSideRegionWidth;

		bool bBallFound = false;
		bool bPartialBallFound = false;

		// For the drive request
		float fLinX = 0.0f;
		float fAngZ = 0.0f;

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

			if (uiCurrentWidth == 0 && bBallFound)
			{
				// Assume no more part of the ball to be found
				break;
			}

			// Find the current width to determine if we need to move forward
			// Also get the ball position
			if (uiCurrentWidth > uiWidth)
			{
				uiWidth = uiCurrentWidth;
				uiBallPos = uiFirstWhitePixelPos + (uiWidth / 2);
			}
		}

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

			if (uiBallPos < uiLeftRegionEnd)
			{
				// Try to quickly roughly centre the ball first before moving forward
				if (uiBallPos < uiLeftRegionEnd - (uiSideRegionWidth * m_fFastRotateSizeFactor))
				{
					fLinX = 0.0f;
					fAngZ = m_fFastRotateSpeed;
				}
				else
				{
					fAngZ = m_fSlowRotateSpeed;
				}
			}
			else if (uiBallPos > uiRightRegionStart)
			{
				// Try to quickly roughly centre the ball first before moving forward
				if (uiBallPos > uiRightRegionStart + (uiSideRegionWidth * m_fFastRotateSizeFactor))
				{
					fLinX = 0.0f;
					fAngZ = -(m_fFastRotateSpeed);
				}
				else
				{
					fAngZ = -(m_fSlowRotateSpeed);
				}
			}
			else
			{

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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "process_image");

	ProcessImageNode obj;

	ros::spin();

	return 0;
}
