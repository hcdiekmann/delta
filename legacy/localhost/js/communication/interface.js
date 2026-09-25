//show input boxes & labels
function showConnectInputUI(){
    document.getElementById('lblAmsID').style.display = "inline-block"; 
    document.getElementById('lblAdsPort').style.display = "inline-block"; 
    document.getElementById('netId').style.display = "inline-block"; 
    document.getElementById('port').style.display = "inline-block";
    document.getElementById('connectBtn').textContent = "Connect";
  }
  
  //hide input boxes & labels
  function hideConnectInputUI(){
    document.getElementById('lblAmsID').style.display = "none"; 
    document.getElementById('lblAdsPort').style.display = "none"; 
    document.getElementById('netId').style.display = "none"; 
    document.getElementById('port').style.display = "none";
    document.getElementById('connectBtn').textContent = "Disconnect";
  }