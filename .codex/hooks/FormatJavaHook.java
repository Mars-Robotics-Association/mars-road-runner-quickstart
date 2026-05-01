import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class FormatJavaHook {
    private static final Pattern FILE_PATH_PATTERN = Pattern.compile("\\\"file_path\\\"\\s*:\\s*\\\"((?:\\\\.|[^\\\"])*)\\\"");

    public static void main(String[] args) {
        try {
            String input = readAll(System.in);
            if (input == null || input.isBlank()) {
                System.exit(0);
                return;
            }

            String filePath = extractFilePath(input);
            if (filePath == null || filePath.isBlank() || !filePath.toLowerCase().endsWith(".java")) {
                System.exit(0);
                return;
            }

            Path target = Paths.get(filePath);
            if (!Files.exists(target)) {
                System.exit(0);
                return;
            }

            Path jarPath = Paths.get(System.getProperty("user.home"), ".claude", "google-java-format.jar");
            if (!Files.exists(jarPath)) {
                System.exit(0);
                return;
            }

            Process process = new ProcessBuilder(
                "java",
                "-jar",
                jarPath.toString(),
                "--aosp",
                "--skip-removing-unused-imports",
                "--replace",
                target.toString()
            ).inheritIO().start();

            process.waitFor();
            System.exit(0);
        } catch (Exception ignored) {
            System.exit(0);
        }
    }

    private static String readAll(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buffer = new byte[4096];
        int n;
        while ((n = in.read(buffer)) != -1) {
            out.write(buffer, 0, n);
        }
        return out.toString(StandardCharsets.UTF_8);
    }

    private static String extractFilePath(String json) {
        Matcher m = FILE_PATH_PATTERN.matcher(json);
        if (!m.find()) return null;
        return unescapeJsonString(m.group(1));
    }

    private static String unescapeJsonString(String s) {
        StringBuilder sb = new StringBuilder(s.length());
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c == '\\' && i + 1 < s.length()) {
                char n = s.charAt(++i);
                switch (n) {
                    case '\\': sb.append('\\'); break;
                    case '"': sb.append('"'); break;
                    case '/': sb.append('/'); break;
                    case 'b': sb.append('\b'); break;
                    case 'f': sb.append('\f'); break;
                    case 'n': sb.append('\n'); break;
                    case 'r': sb.append('\r'); break;
                    case 't': sb.append('\t'); break;
                    case 'u':
                        if (i + 4 < s.length()) {
                            String hex = s.substring(i + 1, i + 5);
                            try {
                                sb.append((char) Integer.parseInt(hex, 16));
                                i += 4;
                            } catch (NumberFormatException e) {
                                sb.append("\\u").append(hex);
                                i += 4;
                            }
                        } else {
                            sb.append("\\u");
                        }
                        break;
                    default:
                        sb.append(n);
                        break;
                }
            } else {
                sb.append(c);
            }
        }
        return sb.toString();
    }
}
