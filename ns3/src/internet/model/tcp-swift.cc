/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 ResiliNets, ITTC, University of Kansas
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Keerthi Ganta <keerthiganta@ku.edu>
 *         Truc Anh N. Nguyen <annguyen@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 */


#include "tcp-swift.h"
#include "tcp-socket-state.h"

#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpSwift");
NS_OBJECT_ENSURE_REGISTERED (TcpSwift);

TypeId
TcpSwift::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpSwift")
    .SetParent<TcpNewReno> ()
    .AddConstructor<TcpSwift> ()
    .SetGroupName ("Internet")
    .AddAttribute ("AlphaMin", "Minimum alpha threshold",
                   DoubleValue (0.3),
                   MakeDoubleAccessor (&TcpSwift::m_alphaMin),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("AlphaMax", "Maximum alpha threshold",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&TcpSwift::m_alphaMax),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("AlphaBase", "Alpha base threshold",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&TcpSwift::m_alphaBase),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BetaMin", "Minimum beta threshold",
                   DoubleValue (0.125),
                   MakeDoubleAccessor (&TcpSwift::m_betaMin),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BetaMax", "Maximum beta threshold",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&TcpSwift::m_betaMax),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BetaBase", "Beta base threshold",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&TcpSwift::m_betaBase),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("WinThresh", "Window threshold",
                   UintegerValue (15),
                   MakeUintegerAccessor (&TcpSwift::m_winThresh),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Theta", "Theta threshold",
                   UintegerValue (5),
                   MakeUintegerAccessor (&TcpSwift::m_theta),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

TcpSwift::TcpSwift (void)
  : TcpNewReno (),
    m_sumRtt (Time (0)),
    m_cntRtt (0),
    m_baseRtt (Time::Max ()),
    m_maxRtt (Time::Min ()),
    m_endSeq (0),
    m_rttAbove (false),
    m_rttLow (0),
    m_alphaMin (0.3),
    m_alphaMax (10.0),
    m_alphaBase (1.0),
    m_alpha (m_alphaMax),
    m_betaMin (0.125),
    m_betaMax (0.5),
    m_betaBase (0.5),
    m_beta (m_betaBase),
    m_winThresh (15),
    m_theta (5),
    m_ackCnt (0)
{
  m_maxCwnd = 10000;
  m_minCwnd = 1;
  m_hops = 1;
  m_hopScale = 1.0;
  m_fSrange = 1000;
  m_baseTarget = 100;
  NS_LOG_FUNCTION (this);
}

TcpSwift::TcpSwift (const TcpSwift& sock)
  : TcpNewReno (sock),
    m_sumRtt (sock.m_sumRtt),
    m_cntRtt (sock.m_cntRtt),
    m_baseRtt (sock.m_baseRtt),
    m_maxRtt (sock.m_maxRtt),
    m_endSeq (sock.m_endSeq),
    m_rttAbove (sock.m_rttAbove),
    m_rttLow (sock.m_rttLow),
    m_alphaMin (sock.m_alphaMin),
    m_alphaMax (sock.m_alphaMax),
    m_alphaBase (sock.m_alphaBase),
    m_alpha (sock.m_alpha),
    m_betaMin (sock.m_betaMin),
    m_betaMax (sock.m_betaMax),
    m_betaBase (sock.m_betaBase),
    m_beta (sock.m_beta),
    m_winThresh (sock.m_winThresh),
    m_theta (sock.m_theta),
    m_ackCnt (sock.m_ackCnt)
{
  m_maxCwnd = 10000;
  m_minCwnd = 1;
  m_hops = 1;
  m_hopScale = 1.0;
  m_fSrange = 1000;
  m_baseTarget = 100;
  NS_LOG_FUNCTION (this);
}

TcpSwift::~TcpSwift (void)
{
  NS_LOG_FUNCTION (this);
}

void
TcpSwift::RecalcParam (uint32_t cWnd)
{
  NS_LOG_FUNCTION (this << cWnd);

  if (cWnd < m_winThresh)
    {
      NS_LOG_INFO ("cWnd < winThresh, set alpha & beta to base values");

      m_alpha = m_alphaBase;
      m_beta = m_betaBase;
    }
  else if (m_cntRtt > 0)
    {
      double dm = static_cast<double> (CalculateMaxDelay ().GetMilliSeconds ());
      double da = static_cast<double> (CalculateAvgDelay ().GetMilliSeconds ());

      NS_LOG_INFO ("Updated to dm = " << dm << " da = " << da);

      CalculateAlpha ();
      CalculateBeta ();
    }
}

void
TcpSwift::CongestionStateSet (Ptr<TcpSocketState> tcb,
                                 const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this << tcb << newState);

  if (newState == TcpSocketState::CA_LOSS)
    {
      m_alpha = m_alphaBase;
      m_beta = m_betaBase;
      m_rttLow = 0;
      m_rttAbove = false;
      Reset (tcb->m_nextTxSequence);
    }
}

void
TcpSwift::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << segmentsAcked);
  //Ready to asses delay
  if (tcb->m_lastAckedSeq >= m_endSeq)
    {
      RecalcParam (tcb->m_cWnd);
      Reset (tcb->m_nextTxSequence);
    }


  if (tcb->m_cWnd < tcb->m_ssThresh)
    {
      TcpNewReno::SlowStart (tcb, segmentsAcked);
      NS_LOG_INFO ("In SlowStart, updated to cwnd " << tcb->m_cWnd <<
                   " ssthresh " << tcb->m_ssThresh);
    }

  //This is the Illinois code, but I'm keeping it in for reference.
  /*else
    {
      uint32_t segCwnd = tcb->GetCwndInSegments ();
      uint32_t oldCwnd = segCwnd;

      if (segmentsAcked > 0)
        {
          m_ackCnt += segmentsAcked * m_alpha;
        }

      while (m_ackCnt >= segCwnd)
        {
          m_ackCnt -= segCwnd;
          segCwnd += 1;
        }

      if (segCwnd != oldCwnd)
        {
          tcb->m_cWnd = segCwnd * tcb->m_segmentSize;
          NS_LOG_INFO ("In CongAvoid, updated to cwnd " << tcb->m_cWnd <<
                       " ssthresh " << tcb->m_ssThresh);
        }
    }
    */
}

void
TcpSwift::TargetDelay(Ptr<TcpSocketState> tcb){
  double minOp = std::min((m_alpha/tcb->m_cWnd.Get())+m_beta, m_fSrange);
  double maxOp = std::max(0.0, minOp);

  m_targetDelay = m_baseTarget + (m_hops * m_hopScale) + maxOp;
  
}

void
TcpSwift::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t packetsAcked,
                        const Time &rtt)
{
  NS_LOG_FUNCTION (this << tcb << packetsAcked << rtt);

  m_retransmitCount = 0;
  TargetDelay(tcb);
  if (rtt.IsZero ())
    {
      return;
    }

  // Keep track of minimum RTT
  m_baseRtt = std::min (m_baseRtt, rtt);

  // Keep track of maximum RTT
  m_maxRtt = std::max (rtt, m_maxRtt);

  ++m_cntRtt;
  m_sumRtt += rtt;

  NS_LOG_INFO ("Updated baseRtt = " << m_baseRtt << " maxRtt = " << m_maxRtt <<
               " cntRtt = " << m_cntRtt << " sumRtt = " << m_sumRtt);


  uint32_t cwnd = tcb->GetCwndInSegments();

  //Additive increase
  if(rtt.GetDouble() < m_targetDelay){
    if(cwnd >= 1){
      cwnd = cwnd + (m_addInc/m_nAcked);
    }
    else{
      cwnd = cwnd + (m_addInc * m_nAcked);
    }
  }

  //Multiplicative decrease
  else{
    if(m_canDecrease){
      cwnd *= std::max(1-(m_beta*((rtt.GetDouble()-m_targetDelay)/rtt.GetDouble())), 1-m_maxDecrease);
    }
  }
  tcb->m_cWnd = cwnd * tcb->m_segmentSize;
}

uint32_t
TcpSwift::GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << tcb << bytesInFlight);

  uint32_t segBytesInFlight = bytesInFlight / tcb->m_segmentSize;
  uint32_t ssThresh = static_cast<uint32_t> (std::max (2.0, (1.0 - m_beta) * segBytesInFlight));

  NS_LOG_DEBUG ("Calculated ssThresh (in segments) = " << ssThresh);

  return ssThresh * tcb->m_segmentSize;
}

void
TcpSwift::CalculateAlpha ()
{
  NS_LOG_FUNCTION (this);

  //m_maxCwnd and m_minCwnd may have to be real max and min
  m_alpha = m_fSrange / ((1/sqrt(m_minCwnd))-(1/sqrt(m_maxCwnd)));


  NS_LOG_INFO ("Updated to alpha = " << m_alpha);
}

void
TcpSwift::CalculateBeta ()
{
  NS_LOG_FUNCTION(this);
  //m_maxCwnd may have to be a real max
  m_beta = -m_alpha/(m_maxCwnd);
 
  NS_LOG_INFO ("Updated to beta = " << m_beta);
}

Time
TcpSwift::CalculateAvgDelay () const
{
  NS_LOG_FUNCTION (this);

  return (m_sumRtt / m_cntRtt - m_baseRtt);
}

Time
TcpSwift::CalculateMaxDelay () const
{
  NS_LOG_FUNCTION (this);

  return (m_maxRtt - m_baseRtt);
}

void
TcpSwift::Reset (const SequenceNumber32 &nextTxSequence)
{
  NS_LOG_FUNCTION (this << nextTxSequence);

  m_endSeq = nextTxSequence;
  m_cntRtt = 0;
  m_sumRtt = Time (0);
}

Ptr<TcpCongestionOps>
TcpSwift::Fork (void)
{
  NS_LOG_FUNCTION (this);

  return CopyObject<TcpSwift> (this);
}

std::string
TcpSwift::GetName () const
{
  NS_LOG_FUNCTION (this);

  return "TcpSwift";
}

} // namespace ns3
