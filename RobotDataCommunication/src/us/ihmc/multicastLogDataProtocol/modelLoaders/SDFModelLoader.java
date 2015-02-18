package us.ihmc.multicastLogDataProtocol.modelLoaders;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.MalformedURLException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import javax.management.IntrospectionException;
import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.robotDataCommunication.VisualizerRobot;
import us.ihmc.utilities.ClassLoaderUtils;

public class SDFModelLoader implements LogModelLoader
{
   private final static String resourceDirectoryLocation = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "resources";
   
   private String modelName;
   private byte[] model;
   private String[] resourceDirectories;
   private byte[] resourceZip;
   
   @Override
   public void load(String modelName, byte[] model, String[] resourceDirectories, byte[] resourceZip)
   {
      this.modelName = modelName;
      this.model = model;
      this.resourceDirectories = resourceDirectories;
      this.resourceZip = resourceZip;
      
   }

   @Override
   public SDFRobot createRobot()
   {
      return new VisualizerRobot(createJaxbSDFLoader().getGeneralizedSDFRobotModel(modelName), null);
   }
   
   public JaxbSDFLoader createJaxbSDFLoader()
   {
      if(resourceZip != null)
      {
         Path resourceDirectory = Paths.get(resourceDirectoryLocation, modelName);
         try
         {
            Files.createDirectories(resourceDirectory);
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
         
         ByteArrayInputStream is = new ByteArrayInputStream(resourceZip);
         ZipInputStream zip = new ZipInputStream(is);
         ZipEntry ze = null;
         try
         {
            while((ze = zip.getNextEntry()) != null)
            {
               Path target = resourceDirectory.resolve(ze.getName());
               Files.createDirectories(target.getParent());
               Files.copy(zip, target, StandardCopyOption.REPLACE_EXISTING);
            }
            zip.close();
            is.close();
         }
         catch (IOException e)
         {
            System.err.println("SDFModelLoader: Cannot load model zip file. Not unpacking robot model.");            
         }
         
         try
         {
            ClassLoaderUtils.addURLToSystemClassLoader(resourceDirectory.toUri().toURL());
         }
         catch (IntrospectionException | MalformedURLException e)
         {
            throw new RuntimeException(e);
         }
      }

      ByteArrayInputStream is = new ByteArrayInputStream(model);
      try
      {
         return new JaxbSDFLoader(is, Arrays.asList(resourceDirectories));
      }
      catch (FileNotFoundException | JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public String getModelName()
   {
      return modelName;
   }

   public byte[] getModel()
   {
      return model;
   }
   public String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   public byte[] getResourceZip()
   {
      return resourceZip;
   }


}