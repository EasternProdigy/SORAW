import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.Arrays;

public class ESP32Controller {

    private Socket socket;
    private PrintWriter out;
    private BufferedReader in;

    public ESP32Controller(String ip, int port) throws IOException {
        socket = new Socket(ip, port);
        out = new PrintWriter(socket.getOutputStream(), true);
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
    }

    public void sendCommand(String command) {
        out.println(command);
    }

    public String receiveResponse() throws IOException {
        return in.readLine();
    }

    public void close() throws IOException {
        in.close();
        out.close();
        socket.close();
    }

    public static void main(String[] args) {
        // Replace with the ESP32's IP and the port you are listening on
        try (ESP32Controller controller = new ESP32Controller("IP", 8080)) {
            controller.sendCommand("SET_PID_PARAMS:2:5:1");
            controller.sendCommand("SET_SETPOINT:90");
            while (true) {
                controller.sendCommand("REQUEST_ATTITUDE");
                String response = controller.receiveResponse();
                double[] attitude = Arrays.stream(response.split(":"))
                                          .mapToDouble(Double::parseDouble)
                                          .toArray();
                // Implement your logic here based on the attitude received
                // For example, you might calculate the new PID output and send a command to adjust the motor
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
