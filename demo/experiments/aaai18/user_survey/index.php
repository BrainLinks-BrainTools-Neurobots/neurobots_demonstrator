<!DOCTYPE html>
<html>
<head>
<title>User Study</title>
<link rel="stylesheet" href="style.css">

</head>
<body>
<?
if (isset($_GET["save"])) {
	$req = array("q1" => 1, "q2" => 1, "q3" => 1, "q4" => 1, "q5" => 1, "q6" => 1, "q7" => 1, "q8" => 1,
			"q9" => 1, "q10" => 1, "q11" => 1, "q12" => 1, "q13" => 1, "q14" => 1, "q15" => 1, "q20" => 1,
			"q21" => 1, "q30" => 1, "log_action" => 1, "log_nav" => 1, "submit_name" => 1);
// 	if (!isset($_POST["q1"]) || !isset($_POST["q2"]) || !isset($_POST["q3"]) || !isset($_POST["q4"]) || !isset($_POST["q5"])
// 			|| !isset($_POST["q6"])	
// 			|| !isset($_POST["q7"]) 
// 			|| !isset($_POST["q8"]) || !isset($_POST["q9"])	|| !isset($_POST["q10"]) || !isset($_POST["q11"])
// 			|| !isset($_POST["12"]) || !isset($_POST["q13"]) || !isset($_POST["q14"]) || !isset($_POST["q15"])
// 			|| !isset($_POST["q20"]) || !isset($_POST["q21"])
// 			|| !isset($_POST["q30"]) || !isset($_POST["log_action"]) || !isset($_POST["log_nav"])) {
	if (sizeof(array_diff_key($req, $_POST)) != 0) {
		var_dump(array_diff_key($req, $_POST));
		die("Missing input");
	}

	$servername = "localhost";
	$username = "web";
	$password = "2kF9niBvCtkIXS74";
	$dbname = "user_survey";

	$conn = new mysqli($servername, $username, $password, $dbname);
	
	// Check connection
	if ($conn->connect_error) {
		die("Connection failed: " . $conn->connect_error);
	} 

	$query = sprintf("INSERT INTO data (
		question1I, question2I, question3I, question4I, question5I, question6I, 
		question7I, question8I, question9I, question10I, question11I, question12I, question13I, question14I, question15I, 
		question1S, question2S, question3S, nav_log, action_log)
	VALUES ('%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', 
				'%s', '%s', '%s', '%s', '%s')",
            (int) $_POST["q1"],
			(int) $_POST["q2"],
			(int) $_POST["q3"],
			(int) $_POST["q4"],
			(int) $_POST["q5"],
			(int) $_POST["q6"],
			(int) $_POST["q7"],
			(int) $_POST["q8"],
			(int) $_POST["q9"],
			(int) $_POST["q10"],
			(int) $_POST["q11"],
			(int) $_POST["q12"],
			(int) $_POST["q13"],
			(int) $_POST["q14"],
			(int) $_POST["q15"],
			mysqli_real_escape_string($conn, $_POST["q20"]),
			mysqli_real_escape_string($conn, $_POST["q21"]),
			mysqli_real_escape_string($conn, $_POST["q30"]),
			mysqli_real_escape_string($conn, $_POST["log_nav"]),
			mysqli_real_escape_string($conn, $_POST["log_action"]));

	if ($conn->query($query) === TRUE) {
		echo "Thank you! <a href=\"index.php\">back</a>";
		
	} else {
		echo "Error: " . $sql . "<br>" . $conn->error;
	}

	$conn->close();
}
else {
?>

<div style="width:1400px" class="wmfg_layout_0">

<form method="post" action="index.php?save">

<ul class="wmfg_questions">
	<h1>User Study</h1>
	
	<li class="wmfg_q">
		<label class="wmfg_label">Did the available options comply with your expectations?</label>
		<table>
		  <tr bgcolor="#e6f2ff">
			<td valign="bottom" width="600"><label class="wmfg_label" for="q1">Scenario 1: Move to garden</label></td>
			<td><div class="controls">
		  <label class="radio">
			<input type="radio" name="q1" id="q1_1" value="5" required>
			Comply
		  </label>
		  <label class="radio">
			 <input type="radio" name="q1" id="q1_2" value="4" required>
		  </label>
		  <label class="radio">
			<input type="radio" name="q1" id="q1_3" value="3" required>
			Understandable
		  </label>
		  <label class="radio">
			<input type="radio" name="q1" id="q1_4" value="2" required>
		  </label>
		  <label class="radio">
			<input type="radio" name="q1" id="q1_5" value="1" required>
			No clue!
		  </label>
		</div></td>
		  </tr>
		  <tr>
			<td><label class="wmfg_label" for="q2">Scenario 2: Drink beer using a beerstein glass</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q2" id="q2_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q2" id="q2_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q2" id="q2_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q2" id="q2_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q2" id="q2_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		  <tr  bgcolor="#e6f2ff">
			<td><label class="wmfg_label" for="q3">Scenario 3: Arrange a red flower in a red vase</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q3" id="q3_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q3" id="q3_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q3" id="q3_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q3" id="q3_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q3" id="q3_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		<tr>
			<td><label class="wmfg_label" for="q4">Scenario 4: Place a red rose on the couch table</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q4" id="q4_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q4" id="q4_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q4" id="q4_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q4" id="q4_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q4" id="q4_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		  <tr bgcolor="#e6f2ff">
			<td><label class="wmfg_label" for="q5">Scenario 5: Give a redwine glass, filled with redwine, to your friend</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q5" id="q5_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q5" id="q5_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q5" id="q5_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q5" id="q5_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q5" id="q5_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		</table>
		
	</li>
	
	<li class="wmfg_q">
		<label class="wmfg_label">How intuitive is the control of the GUI?</label>
		<table class="wmfg_answers">
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q6" value="5" required /></td>
				<td><label class="wmfg_label_a" for="radio_1">Excellent</label></td>
			</tr>
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q6" value="4" required /></td>
				<td><label class="wmfg_label_a" for="radio_2">Intuitive</label></td>
			</tr>
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q6" value="3" required /></td>
				<td><label class="wmfg_label_a" for="radio_3">Acceptable</label></td>
			</tr>
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q6" value="2" required /></td>
				<td><label class="wmfg_label_a" for="radio_3">Inconvinient</label></td>
			</tr>
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q6" value="1" required /></td>
				<td><label class="wmfg_label_a" for="radio_3">Not intuitive</label></td>
			</tr>
		</table>
	</li>

	<li class="wmfg_q">
		<label class="wmfg_label">Would you prefer to access objects by their internal name instead of using references ("v1" vs. "green vase in the kitchen")?</label>
		<table class="wmfg_answers">
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q7" value="1" required/></td>
				<td><label class="wmfg_label_a" for="radio_1">Yes</label></td>
			</tr>
			<tr class="wmfg_a">
				<td class="wmfg_a_td"><input type="radio" class="wmfg_radio" name="q7" value="0" required/></td>
				<td><label class="wmfg_label_a" for="radio_2">No</label></td>
			</tr>
		</table>
	</li>

	<p>Please use this scenario in the following questions:</p>
	<p><img src="images/screenshot1.png" alt="Scenario" width="1350"></p>
	
	<li class="wmfg_q">
		<label class="wmfg_label" for="textarea_id">You are in the living room. How would you refer to the objects in the following scenario by natural language?</label>
		
		<label class="wmfg_label" for="q20">v2:</label>
		<textarea class="wmfg_textarea" name="q20" id="q20" style="height:80px" maxlength="2000" minlength="0" required></textarea>
		<label class="wmfg_label" for="q21">g6:</label>
		<textarea class="wmfg_textarea" name="q21" id="q20" style="height:80px" maxlength="2000" minlength="0" required></textarea>
	</li>
	
	<li class="wmfg_q">
	<label class="wmfg_label" for="textarea_id">Given the following references for v2, which one would you prefer (you are in the living room):</label>
		<table>
		  <tr bgcolor="#e6f2ff">
			<td valign="bottom" width="300"><label class="wmfg_label" for="q8">The green vase</label></td>
			<td><div class="controls">
		  <label class="radio">
			<input type="radio" name="q8" id="q8_1" value="5" required>
			Highly prefer
		  </label>
		  <label class="radio">
			 <input type="radio" name="q8" id="q8_2" value="4" required>
		  </label>
		  <label class="radio">
			<input type="radio" name="q8" id="q8_3" value="3" required>
			OK
		  </label>
		  <label class="radio">
			<input type="radio" name="q8" id="q8_4" value="2" required>
		  </label>
		  <label class="radio">
			<input type="radio" name="q8" id="q8_5" value="1" required>
			Not prefer at all
		  </label>
		</div></td>
		  </tr>
		  <tr>
			<td><label class="wmfg_label" for="q9">The vase containing a flower</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q9" id="q9_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q9" id="q9_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q9" id="q9_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q9" id="q9_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q9" id="q9_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		  <tr  bgcolor="#e6f2ff">
			<td><label class="wmfg_label" for="q10">The green vase which is in the right shelf in the pantry which contains a red rose</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q10" id="q10_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q10" id="q10_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q10" id="q10_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q10" id="q10_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q10" id="q10_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		<tr>
			<td><label class="wmfg_label" for="q11">The vase in the right shelf</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q11" id="q11_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q11" id="q11_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q11" id="q11_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q11" id="q11_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q11" id="q11_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		</table>
	</li>
	
	<li class="wmfg_q">
	<label class="wmfg_label" for="textarea_id">Given the following references for g6, which one would you prefer (you are in the living room):</label>
		<table>
		  <tr bgcolor="#e6f2ff">
			<td valign="bottom" width="300"><label class="wmfg_label" for="q12">The red-wine glass containing red wine on the couch-table in the living-room</label></td>
			<td><div class="controls">
		  <label class="radio">
			<input type="radio" name="q12" id="q12_1" value="5" required>
			Highly prefer
		  </label>
		  <label class="radio">
			 <input type="radio" name="q12" id="q12_2" value="4" required>
		  </label>
		  <label class="radio">
			<input type="radio" name="q12" id="q12_3" value="3" required>
			OK
		  </label>
		  <label class="radio">
			<input type="radio" name="q12" id="q12_4" value="2" required>
		  </label>
		  <label class="radio">
			<input type="radio" name="q12" id="q12_5" value="1" required>
			Not prefer at all
		  </label>
		</div></td>
		  </tr>
		  <tr>
			<td><label class="wmfg_label" for="q13">The red-wine glass in the living-room</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q13" id="q13_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q13" id="q13_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q13" id="q13_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q13" id="q13_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q13" id="q13_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		  <tr  bgcolor="#e6f2ff">
			<td><label class="wmfg_label" for="q14">The glass containing red wine on the couch table</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q14" id="q14_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q14" id="q14_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q14" id="q14_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q14" id="q14_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q14" id="q14_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		<tr>
			<td><label class="wmfg_label" for="q15">The red-wine glass</label></td>
			<td><div class="controls">
		  <label class="radio2">
			<input type="radio" name="q15" id="q15_1" value="5" required>
		  </label>
		  <label class="radio2">
			 <input type="radio" name="q15" id="q15_2" value="4" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q15" id="q15_3" value="3" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q15" id="q15_4" value="2" required>
		  </label>
		  <label class="radio2">
			<input type="radio" name="q15" id="q15_5" value="1" required>
		  </label>
		</div></td>
		  </tr>
		</table>
	</li>

	<li class="wmfg_q">
		<label class="wmfg_label" for="q30">Optional Comments about the GUI and planning process:</label>
		<textarea class="wmfg_textarea" name="q30" id="q30" style="height:80px" maxlength="2000" minlength="0"></textarea>
	</li>

	<li class="wmfg_q wmfg_q_red">
		<label class="wmfg_label" for="textarea_id3">Action LOG-Files:</label>
		<textarea class="wmfg_textarea" name="log_action" id="textarea_id3" style="height:80px" maxlength="2000" minlength="0" required></textarea>
	</li>

	<li class="wmfg_q wmfg_q_red">
		<label class="wmfg_label" for="textarea_id4">Navigation LOG-Files:</label>
		<textarea class="wmfg_textarea" name="log_nav" id="textarea_id4" style="height:80px" maxlength="2000" minlength="0" required></textarea>
	</li>

	<li class="wmfg_q">
		<input type="submit" class="wmfg_btn" name="submit_name" id="submit_id" value="Submit" />
	</li>

</ul>

</form>

</div>

<?
}
?>
</body>
</html>
